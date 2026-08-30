/* sysex_assembler.h — reassemble USB-MIDI SysEx packets into whole messages.
 *
 * Cable 2 delivers SysEx as a run of 4-byte USB-MIDI packets: CIN 0x04 carries
 * three bytes and says "more follows", and CIN 0x05/0x06/0x07 carry one, two or
 * three bytes and say "this is the end". Nothing on the chain path has ever put
 * those back together — the two cable-2 dispatchers skip every CIN below 0x08 —
 * so a chain slot could not see a device's answer at all.
 *
 * ONE ASSEMBLER PER PORT, NOT PER SLOT. There is a single byte stream, and
 * whose message it is cannot be known until it has been read. So the host
 * assembles once and then decides who to hand it to; a per-slot assembler would
 * be four copies of the same partial message.
 *
 * WHY A FIXED BUFFER THAT DROPS. This runs on the SPI callback, where the rule
 * is the one stated in ext_midi_ring.h: no allocation, no I/O, no locks, no
 * clock. A message longer than SYSEX_ASSEMBLER_MAX is therefore DISCARDED
 * WHOLE and counted, never truncated and delivered — a half patch dump that
 * looks complete is worse than no patch dump, because a receiver has no way to
 * tell it was cut. The cap is sized for parameter and patch-level SysEx (a
 * D-50 patch dump is ~450 bytes); multi-kilobyte bulk dumps are out of scope
 * and are reported through `oversize` rather than silently mangled.
 *
 * INTERLEAVING IS DETECTED, NOT REPAIRED. Two devices dumping at once — a hub
 * with a QY-70 and a D-50 on it — interleave in the stream, and USB-MIDI
 * carries no tag saying which device a cable-2 packet came from, so the two
 * runs cannot be separated. An F0 arriving while a message is already open is
 * the signal that this has happened: the message in progress is abandoned and
 * the new one started, which loses one message rather than splicing two into a
 * plausible-looking third. Counted in `aborts`. (A MIDI merge box is the SAFER
 * topology here, not the more dangerous one — a competent merger holds one
 * message until F7 before letting the other through, precisely because
 * interleaved SysEx breaks receivers. A hub does no such thing.)
 *
 * Pure: no allocation, no I/O, no locks, no clock. SPI-callback-safe, and
 * host-testable without a device (tests/host/test_sysex_assembler.c).
 */

#ifndef SYSEX_ASSEMBLER_H
#define SYSEX_ASSEMBLER_H

#include <stdint.h>
#include <string.h>

/* Bytes. Holds F0 .. F7 inclusive. See the note above on why this drops rather
 * than truncates, and why bulk dumps are out of scope. */
#define SYSEX_ASSEMBLER_MAX 512

/* USB-MIDI Code Index Numbers that carry SysEx payload. */
#define SYSEX_CIN_START     0x04   /* start or continue, 3 bytes */
#define SYSEX_CIN_END_1     0x05   /* ends with 1 byte (also single-byte common) */
#define SYSEX_CIN_END_2     0x06   /* ends with 2 bytes */
#define SYSEX_CIN_END_3     0x07   /* ends with 3 bytes */

typedef struct {
    uint8_t  buf[SYSEX_ASSEMBLER_MAX];
    uint16_t len;         /* bytes held; 0 when idle */
    uint8_t  open;        /* 1 = a message is in progress */
    uint8_t  dropping;    /* 1 = current message overran; discard to its end */
    uint32_t completed;   /* messages delivered whole */
    uint32_t aborts;      /* abandoned: interleaved F0, or a stray end packet */
    uint32_t oversize;    /* discarded for exceeding SYSEX_ASSEMBLER_MAX */
} sysex_assembler_t;

static inline void sysex_assembler_reset(sysex_assembler_t *a) {
    if (a) memset(a, 0, sizeof(*a));
}

/* Abandon whatever is in progress WITHOUT touching the counters. For a port
 * that has gone away, or a mode change that makes the partial message
 * meaningless. */
static inline void sysex_assembler_discard(sysex_assembler_t *a) {
    if (!a) return;
    a->len = 0;
    a->open = 0;
    a->dropping = 0;
}

/* True for the CINs this assembler consumes. Callers use it to decide whether a
 * packet is theirs before doing anything else with it. */
static inline int sysex_cin_is_sysex(uint8_t cin) {
    return cin >= SYSEX_CIN_START && cin <= SYSEX_CIN_END_3;
}

/* How many payload bytes a SysEx CIN carries. 0 for anything else.
 *
 * CIN 0x05 IS AMBIGUOUS in USB-MIDI: it means "SysEx ends with one byte" AND
 * "single-byte System Common". Disambiguated by state, not by the byte —
 * treated as an end only while a message is open. A 0x05 arriving idle is a
 * System Common message that is not ours, and returning 1 for it here would
 * make the caller feed a byte into a message that does not exist. */
static inline int sysex_cin_bytes(uint8_t cin) {
    switch (cin) {
        case SYSEX_CIN_START: return 3;
        case SYSEX_CIN_END_1: return 1;
        case SYSEX_CIN_END_2: return 2;
        case SYSEX_CIN_END_3: return 3;
        default:              return 0;
    }
}

/*
 * Feed one USB-MIDI packet.
 *
 * Returns the LENGTH of a complete message (>0) when this packet finished one,
 * in which case `a->buf` holds it from F0 through F7 inclusive and stays valid
 * until the next call. Returns 0 otherwise — mid-message, ignored, or
 * discarded.
 *
 * The caller passes the CIN and the three payload bytes as they sit in the
 * mailbox. It does NOT need to know which CIN ends a message.
 */
static inline int sysex_assembler_feed(sysex_assembler_t *a, uint8_t cin,
                                       uint8_t b0, uint8_t b1, uint8_t b2) {
    if (!a) return 0;
    if (!sysex_cin_is_sysex(cin)) return 0;

    const uint8_t bytes[3] = { b0, b1, b2 };
    int n = sysex_cin_bytes(cin);

    /*
     * A message opens on F0 and only on F0.
     *
     * An end-packet arriving with nothing open is a message whose start we
     * missed — the boot case, or the tail of one the previous mode dropped.
     * Counting it as an abort is the honest reading: there WAS a message and
     * we do not have it. Silently ignoring it would make a lost start
     * indistinguishable from an idle port.
     */
    if (!a->open) {
        if (bytes[0] != 0xF0) {
            /*
             * A STATUS byte here is not ours and not an error. CIN 0x05 is
             * ambiguous in USB-MIDI — "SysEx ends with one byte" AND
             * "single-byte System Common" — so an idle 0x05 carrying F6 (tune
             * request) or a stray F7 is a real message that simply is not
             * SysEx. Counting those as aborts would bury a genuine lost start
             * under traffic that is working exactly as intended.
             *
             * A DATA byte (bit 7 clear) with nothing open is the real thing
             * this counter is for: payload whose F0 we never saw.
             */
            if (!(bytes[0] & 0x80)) a->aborts++;
            return 0;
        }
        a->len = 0;
        a->dropping = 0;
        a->open = 1;
    } else if (bytes[0] == 0xF0 && cin == SYSEX_CIN_START) {
        /*
         * INTERLEAVED START. Two devices are talking over each other (or a
         * device restarted mid-message). Abandon what we have and take the new
         * one: splicing them would produce a message that parses and is wrong,
         * which is the failure this branch exists to prevent.
         *
         * Only checked on a START packet. An F0 byte appearing as PAYLOAD is
         * impossible — every SysEx data byte has bit 7 clear — so this cannot
         * fire on legitimate content.
         */
        a->aborts++;
        a->len = 0;
        a->dropping = 0;
    }

    for (int i = 0; i < n; i++) {
        uint8_t byte = bytes[i];

        if (!a->dropping) {
            if (a->len >= SYSEX_ASSEMBLER_MAX) {
                /* Overran. Keep consuming to find this message's F7 so the
                 * NEXT one starts clean, but deliver nothing. */
                a->dropping = 1;
                a->oversize++;
            } else {
                a->buf[a->len++] = byte;
            }
        }

        if (byte == 0xF7) {
            int out = a->dropping ? 0 : (int)a->len;
            a->open = 0;
            a->dropping = 0;
            if (out > 0) {
                a->completed++;
                return out;
            }
            a->len = 0;
            return 0;
        }
    }

    return 0;
}

#endif /* SYSEX_ASSEMBLER_H */
