#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/../.."

# SysEx reaches a chain slot's synth ONLY if it asked, and never falls through
# into the voice path.
#
# The unit test covers reassembly. This covers the two wiring properties that
# reassembly cannot, and that are silent when wrong:
#
#   1. THE GATE. No sub-plugin has ever been handed a message that was not 1 or
#      3 bytes -- the chain host forwards a hardcoded three -- so a module's
#      handling of a long one has never been exercised. Dropping the opt-in
#      would start feeding every chain module 400-byte dumps on the SPI
#      callback, and nothing would fail loudly. It is also what makes this
#      change inert: with no module declaring sysex_in, behaviour is unchanged.
#
#   2. THE EARLY RETURN. Everything after the gate in v2_on_midi reads msg[0]
#      as a channel-voice status byte. 0xF0 is not a voice type, but msg[1] and
#      msg[2] of a SysEx message are manufacturer and address bytes that the
#      knob-CC handler would read as a CC number and value. A fall-through
#      would not crash; it would quietly move a knob.

fail() { echo "FAIL: $1" >&2; exit 1; }

# ---- 1. the host broadcasts only what the assembler completed -------------
host="src/host/shadow_midi.c"
[ -f "$host" ] || fail "$host is gone"

blk=$(awk '/^void shadow_dispatch_cable2_sysex_slots/,/^}/' "$host")
[ -n "$blk" ] || fail "shadow_dispatch_cable2_sysex_slots is gone from $host"

command grep -q "sysex_cin_is_sysex(cin)" <<<"$blk" || \
  fail "the dispatcher no longer filters on the SysEx CIN predicate"
command grep -q "cable != 0x02" <<<"$blk" || \
  fail "the dispatcher no longer restricts itself to cable 2"
command grep -q "sysex_assembler_feed" <<<"$blk" || \
  fail "the dispatcher no longer reassembles -- it would ship raw 3-byte chunks"
command grep -q "if (len <= 0) continue;" <<<"$blk" || \
  fail "the dispatcher dispatches without checking a message completed"

# The dedup must key on the full 8-byte slot. A content-only key would drop the
# second of two packets carrying identical payload -- a run of zeros in a patch
# dump -- and corrupt the message rather than lose it.
command grep -q "g_sysex_dedup" <<<"$blk" || \
  fail "the SysEx scan does not dedup; a dump would be re-fed every SPI frame"
command grep -q "&in_src\[i\]" <<<"$blk" || \
  fail "the dedup key is not the full 8-byte slot (timestamp included)"

# Its own ring. Sharing one with the voice dispatchers lets a voice event evict
# a SysEx packet's entry, which re-feeds that packet and corrupts the message.
command grep -q "static event_dedup_entry_t g_sysex_dedup" "$host" || \
  fail "g_sysex_dedup is not a dedicated ring"

# ---- 2. it runs only when no tool owns the surface ------------------------
shim="src/schwung_shim.c"
command grep -q "shadow_dispatch_cable2_sysex_slots();" "$shim" || \
  fail "the shim never calls the SysEx dispatcher"

# In overtake mode the tool already receives the whole cable-2 stream, SysEx
# included. Calling this there as well would deliver every message twice.
gate=$(awk '/overtake_mode == 0 \|\| shadow_control->suspend_overtake/,/^    }/' "$shim")
command grep -q "shadow_dispatch_cable2_sysex_slots" <<<"$gate" || \
  fail "the SysEx dispatcher is called outside the no-tool-active gate"

# ---- 3. the chain gates on the declared capability -----------------------
midi="src/modules/chain/dsp/chain_midi.c"
blk=$(awk '/void v2_on_midi/,/chain_update_clock_runtime/' "$midi")
[ -n "$blk" ] || fail "v2_on_midi is gone from $midi"

command grep -q "msg\[0\] == 0xF0" <<<"$blk" || \
  fail "v2_on_midi no longer recognises SysEx before the voice path"
command grep -q "inst->synth_sysex_in" <<<"$blk" || \
  fail "THE GATE IS GONE: SysEx would reach every synth, opted in or not"

# The early return, and it must be unconditional -- taken whether or not the
# synth opted in, or a dropped message falls into the knob-CC handler.
returns=$(awk '/if \(msg\[0\] == 0xF0\)/,/^    }/' "$midi" | command grep -c "return;" || true)
[ "$returns" -ge 1 ] || \
  fail "SysEx falls through into the voice path instead of returning"

# ---- 4. the capability defaults off and is read as a BOOLEAN --------------
host_c="src/modules/chain/dsp/chain_host.c"
command grep -q "inst->synth_sysex_in = 0;" "$host_c" || \
  fail "synth_sysex_in is not defaulted off at load -- a stale slot would inherit it"

# json_get_int would mis-parse it: atoi("true") is 0, so the capability would
# silently never take effect. Same trap the audio_in check documents above it.
command grep -q 'json_get_bool_in_section(json, "capabilities", "sysex_in"' "$host_c" || \
  fail "sysex_in is not read with json_get_bool_in_section (atoi(\"true\") == 0)"

# ---- 5. the assembler stays pure ----------------------------------------
# It runs on the SPI callback. The house rule for this layer is stated in
# ext_midi_ring.h: no allocation, no I/O, no locks, no clock.
asm_h="src/host/sysex_assembler.h"
for banned in malloc calloc realloc free fopen printf usleep pthread_ clock_gettime; do
  if command grep -q "$banned" "$asm_h"; then
    fail "sysex_assembler.h calls $banned -- it runs on the SPI callback"
  fi
done

# Oversize must DISCARD, not truncate. A truncated dump still parses, which is
# how a module ends up acting on half a patch.
command grep -q "oversize" "$asm_h" || \
  fail "the assembler no longer tracks oversize messages"

echo "PASS: sysex_in opt-in wiring"
