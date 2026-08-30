/*
 * Can a chain slot be handed a whole SysEx message, and never half of one?
 *
 * Cable 2 delivers SysEx as a run of 4-byte USB-MIDI packets and nothing on
 * the chain path has ever put them back together. The assembler is what does,
 * and every property worth pinning here is one that is SILENT when wrong: a
 * message that is delivered truncated still parses, an interleaved pair still
 * looks like a message, and a re-fed buffer still returns a plausible length.
 *
 * The properties:
 *   1. a message split across packets comes back whole, F0 through F7;
 *   2. every packet boundary the 4 SysEx CINs can produce is handled, so a
 *      message ending 1, 2 or 3 bytes into its last packet is not lost;
 *   3. an interleaved F0 ABANDONS rather than splices — the failure that
 *      would otherwise hand a module a message that parses and is wrong;
 *   4. oversize DISCARDS WHOLE and never delivers a prefix;
 *   5. the assembler recovers: the message AFTER a failure is clean;
 *   6. an idle System Common byte is not miscounted as a lost message.
 */
#include <stdio.h>
#include <string.h>

#include "sysex_assembler.h"

static int failures = 0;

static void check(int cond, const char *what) {
    if (cond) {
        printf("PASS %s\n", what);
    } else {
        printf("FAIL %s\n", what);
        failures++;
    }
}

/* Feed a whole byte string as the host would: CIN 0x04 for every full group of
 * three, then the end CIN matching the remainder. Returns the length the
 * assembler reported, or 0. */
static int feed_message(sysex_assembler_t *a, const uint8_t *bytes, int n) {
    int out = 0;
    int i = 0;
    while (n - i > 3) {
        int r = sysex_assembler_feed(a, SYSEX_CIN_START,
                                     bytes[i], bytes[i + 1], bytes[i + 2]);
        if (r > 0) out = r;
        i += 3;
    }
    int rem = n - i;
    uint8_t cin = (rem == 1) ? SYSEX_CIN_END_1
                : (rem == 2) ? SYSEX_CIN_END_2
                             : SYSEX_CIN_END_3;
    int r = sysex_assembler_feed(a, cin,
                                 bytes[i],
                                 rem > 1 ? bytes[i + 1] : 0,
                                 rem > 2 ? bytes[i + 2] : 0);
    if (r > 0) out = r;
    return out;
}

int main(void) {
    sysex_assembler_t a;

    /* ---- 1. a message split across packets comes back whole ---- */
    /* A Yamaha XG identity reply shape: F0 43 ... F7, 8 bytes -> 2 START
     * packets plus a 2-byte end. */
    {
        const uint8_t id[] = { 0xF0, 0x43, 0x00, 0x4C, 0x00, 0x00, 0x7E, 0xF7 };
        sysex_assembler_reset(&a);
        int len = feed_message(&a, id, (int)sizeof(id));
        check(len == (int)sizeof(id), "a split message returns its whole length");
        check(memcmp(a.buf, id, sizeof(id)) == 0,
              "and the bytes are the message, F0 through F7 inclusive");
        check(a.completed == 1 && a.aborts == 0 && a.oversize == 0,
              "counted as one clean completion");
    }

    /* ---- 2. every end-CIN boundary ---- */
    /* The remainder after the F0 packet decides which CIN ends the message,
     * and a 1-byte tail is the one an assembler that only handles 0x07 would
     * silently swallow. Lengths 4, 5 and 6 cover ends of 1, 2 and 3. */
    for (int n = 4; n <= 6; n++) {
        uint8_t m[8];
        m[0] = 0xF0;
        for (int i = 1; i < n - 1; i++) m[i] = (uint8_t)(0x10 + i);
        m[n - 1] = 0xF7;
        sysex_assembler_reset(&a);
        int len = feed_message(&a, m, n);
        char what[96];
        snprintf(what, sizeof(what),
                 "a %d-byte message ending on CIN 0x%02X survives", n,
                 (n - 3 == 1) ? SYSEX_CIN_END_1
                 : (n - 3 == 2) ? SYSEX_CIN_END_2 : SYSEX_CIN_END_3);
        check(len == n && memcmp(a.buf, m, (size_t)n) == 0, what);
    }

    /* A message that fits in ONE packet, F0 xx F7 with CIN 0x07. */
    {
        sysex_assembler_reset(&a);
        int len = sysex_assembler_feed(&a, SYSEX_CIN_END_3, 0xF0, 0x7E, 0xF7);
        check(len == 3, "a message complete inside one packet is delivered");
    }

    /* ---- 3. an interleaved F0 abandons, it does not splice ---- */
    /*
     * Two devices on a hub, dumping at once. Splicing them would produce a
     * message that parses and is wrong, which is worse than losing one, so the
     * open message is abandoned and the new one started.
     */
    {
        sysex_assembler_reset(&a);
        sysex_assembler_feed(&a, SYSEX_CIN_START, 0xF0, 0x41, 0x10);  /* Roland opens */
        sysex_assembler_feed(&a, SYSEX_CIN_START, 0xF0, 0x43, 0x00);  /* Yamaha cuts in */
        int len = sysex_assembler_feed(&a, SYSEX_CIN_END_2, 0x01, 0xF7, 0x00);
        check(a.aborts == 1, "the interrupted message is counted as an abort");
        /* F0 43 00 from the start packet, then 01 F7 = five bytes. NOT the
         * eight it would be with the abandoned Roland opener spliced on. */
        check(len == 5, "and the length is the NEW message only, not the splice");
        check(a.buf[0] == 0xF0 && a.buf[1] == 0x43,
              "which starts at the second F0: 43 (Yamaha), not 41 (Roland)");
        check(a.completed == 1, "one completion, not two");
    }

    /* ---- 4. oversize discards whole, never a prefix ---- */
    {
        sysex_assembler_reset(&a);
        sysex_assembler_feed(&a, SYSEX_CIN_START, 0xF0, 0x41, 0x10);
        int delivered = 0;
        /* Well past the cap. Data bytes only — bit 7 clear, so no accidental
         * F7 ends it early. */
        for (int i = 0; i < SYSEX_ASSEMBLER_MAX; i++) {
            int r = sysex_assembler_feed(&a, SYSEX_CIN_START, 0x01, 0x02, 0x03);
            if (r > 0) delivered = r;
        }
        check(delivered == 0, "an over-long message delivers NOTHING mid-flight");
        check(a.oversize == 1, "and is counted as oversize exactly once");

        int r = sysex_assembler_feed(&a, SYSEX_CIN_END_1, 0xF7, 0, 0);
        check(r == 0, "its F7 does not deliver the truncated prefix either");
        check(a.completed == 0, "nothing was ever completed");
    }

    /* ---- 5. the assembler recovers after a failure ---- */
    /*
     * The property that makes 3 and 4 survivable. A dropped message must cost
     * exactly one message: if the assembler stayed wedged, one bad dump would
     * take the port down until reboot.
     */
    {
        const uint8_t good[] = { 0xF0, 0x43, 0x76, 0xF7 };
        int len = feed_message(&a, good, (int)sizeof(good));
        check(len == 4, "the message after an oversize discard is delivered");
        check(memcmp(a.buf, good, sizeof(good)) == 0,
              "and carries none of the discarded message's bytes");
    }
    {
        sysex_assembler_reset(&a);
        sysex_assembler_feed(&a, SYSEX_CIN_START, 0xF0, 0x41, 0x10);
        sysex_assembler_feed(&a, SYSEX_CIN_START, 0xF0, 0x43, 0x00);  /* abort */
        sysex_assembler_feed(&a, SYSEX_CIN_END_1, 0xF7, 0, 0);
        const uint8_t good[] = { 0xF0, 0x41, 0x11, 0xF7 };
        int len = feed_message(&a, good, (int)sizeof(good));
        check(len == 4, "and the message after an interleave abort is delivered");
    }

    /* ---- 6. an idle System Common byte is not a lost message ---- */
    /*
     * CIN 0x05 is ambiguous: "SysEx ends with one byte" AND "single-byte
     * System Common". An idle 0x05 carrying F6 (tune request) is a real
     * message that simply is not ours. Counting it as an abort would bury a
     * genuine lost start under traffic that is working correctly.
     */
    {
        sysex_assembler_reset(&a);
        sysex_assembler_feed(&a, SYSEX_CIN_END_1, 0xF6, 0, 0);
        check(a.aborts == 0, "an idle tune request is ignored, not counted");

        /* A DATA byte with nothing open is the real thing that counter is for:
         * payload whose F0 we never saw. */
        sysex_assembler_feed(&a, SYSEX_CIN_START, 0x01, 0x02, 0x03);
        check(a.aborts == 1, "but orphaned payload IS counted as a lost start");
    }

    /* ---- the CIN predicate matches what the dispatcher filters on ---- */
    {
        check(!sysex_cin_is_sysex(0x03) && sysex_cin_is_sysex(0x04) &&
              sysex_cin_is_sysex(0x07) && !sysex_cin_is_sysex(0x08),
              "sysex_cin_is_sysex covers exactly 0x04-0x07");
    }

    if (failures) {
        printf("\n%d FAILED\n", failures);
        return 1;
    }
    printf("\nALL PASS\n");
    return 0;
}
