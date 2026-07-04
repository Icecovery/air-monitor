# Reverse-Engineering Walkthrough — How the TVOC Firmware Was Cracked

A teaching writeup of the actual process used to (1) get code off a locked-down little
sensor MCU over SWD, and (2) reverse-engineer its firmware far enough to extract the exact
TVOC equation and explain why the sensor was stuck. Written for someone learning embedded
reverse engineering. It includes the **dead ends**, because those are where most of the
learning is.

Target recap: Waveshare TVOC sensor = MOX gas element + **Puya PY32F003F18P6** (ARM
Cortex-M0+, 64 KB flash, 8 KB RAM), talking UART. Symptom: computed TVOC frozen at
~0.352 ppm while the raw ADC still responded to air.

---

## 0. The mental model first (read this before the tools)

Reverse engineering embedded firmware is a loop of **three questions**:

1. **Can I get the bytes?** (acquisition — dump the flash)
2. **What is this code?** (static analysis — disassemble, find structure)
3. **What does it actually do at runtime?** (dynamic analysis — run it, watch memory)

Beginners over-focus on #2. The big wins in this project came from bouncing between #2
and #3: static analysis told me *where* to look, and dynamic analysis (a live chip on a
debugger) told me *what the values really were*, which is 10× faster than tracing math by
hand. **If you can run the target under a debugger, do — don't reverse blind.**

Two guiding principles that saved the most time:

- **Don't reverse what you can recognize.** ~70% of any firmware is library code
  (vendor HAL, CMSIS startup, the compiler's soft-float runtime). Identify and skip it;
  spend effort only on the application-specific parts.
- **Anchor, don't boil the ocean.** Never start at instruction 0 and read forward. Find
  an *anchor* tied to your goal (an address, a constant, a peripheral register, a known
  value in RAM) and expand outward from there.

---

## 1. Tooling inventory (what, and why that one)

| Tool | Role | Why this one |
|---|---|---|
| **OpenOCD** (0.12) | SWD debugger / flash programmer | Free, scriptable (TCL), and its `bcm2835gpio` driver bit-bangs SWD straight off Raspberry Pi GPIO — no hardware probe needed. |
| **Raspberry Pi GPIO** | The SWD "probe" | We had no J-Link/ST-Link. A Pi can bit-bang the 2-wire SWD protocol. Slow (100 kHz) but totally sufficient. |
| **Capstone** (Python) | Disassembler engine | Scriptable disassembly. Unlike a plain `objdump` dump, I could programmatically follow calls, resolve literal pools, and annotate — essential for tracing dataflow. |
| **Python + struct** | Binary surgery & math | Scanning the dump for byte patterns, decoding IEEE-754 floats/doubles, validating the extracted formula numerically. |
| **pyserial** | Talk to the sensor's UART | To capture the real protocol frames and later verify the fix. |
| **GNU objdump / gdb** | Present on the Pi as backup | objdump can disassemble too, but it mangles data (see the dead end in §5). |

**What I did NOT use, but you probably should: Ghidra or IDA.** I hand-traced with
Capstone because I was working over SSH targeting one specific equation. For anything
bigger, a decompiler that produces C-like pseudocode is a different difficulty class.
More on this in §8.

---

## 2. Phase 1 — Acquisition: getting the firmware off the chip

### 2.1 Wire up SWD
SWD is just two signals: **SWDIO** (data) and **SWCLK** (clock), plus GND. The sensor
exposed a 4-pad header (V/G/D/C = VCC/GND/SWDIO/SWCLK). We measured the `V` pad at 3.3 V,
which matches the Pi's GPIO logic level, so we could wire GPIO **directly, no level
shifter**. (Always check target logic voltage first — 5 V into a 3.3 V Pi pin destroys it.)

### 2.2 The OpenOCD config
The one non-obvious value: OpenOCD's bit-bang driver needs the SoC's **peripheral base
address**, which is chip-specific. For the Pi Zero's BCM2835 it's `0x20000000`. Wrong
value = silent failure. (`py32_swd.cfg` in this folder is the working config.)

```
adapter driver bcm2835gpio
bcm2835gpio peripheral_base 0x20000000
adapter gpio swclk 25
adapter gpio swdio 24
transport select swd
adapter speed 100                 # start slow; raise once stable

# Generic Cortex-M target — enough to connect and read memory:
swd newdap py32 cpu -enable
dap create py32.dap -chain-position py32.cpu
target create py32.cpu cortex_m -dap py32.dap
```

### 2.3 First contact
```
sudo openocd -f py32_swd.cfg -c "init; dap info; exit"
```
Success looks like: `SWD DPIDR 0x0bc11477` and `Cortex-M0+ r0p1 processor detected`.
The DPIDR is the debug port's ID — if you see a sane value and the core is detected, the
electrical + protocol layer works.

### 2.4 The read-protection (RDP) gate — and how to test it *safely*
Most MCUs can lock flash from being read out (RDP / readout protection). **Before doing
anything destructive, the dump itself is the RDP test:**
```
sudo openocd -f py32_swd.cfg -c "init; halt; dump_image backup.bin 0x08000000 0x10000; exit"
```
Then validate the dump is *real code*, not a locked/blank read. The trick: a valid ARM
image has a **vector table** at offset 0:
- word[0] = initial stack pointer → should point into RAM (`0x20000xxx`)
- word[1] = reset vector → should point into flash (`0x0800xxxx`)

Ours read `SP=0x20000A50`, `reset=0x080000D5` → real firmware, **RDP off.** If it had
been all `0x00`/`0xFF`/garbage, that means locked, and the honest move is to *stop*:
clearing RDP mass-erases the chip, destroying the firmware you can't back up.

**Lesson:** always dump twice and compare hashes. Two identical MD5s = your reader is
reliable and deterministic. A flaky reader that "sometimes works" will waste hours later.

---

## 3. Phase 2 — Writing flash on a chip with no OpenOCD flash driver

(You can skip this if you only want to *read*. Included because doing it by hand teaches
a lot about how flash controllers work.)

Stock OpenOCD has no PY32 flash algorithm. But flash controllers are just memory-mapped
registers, and OpenOCD can poke registers with `mww` (memory-write-word). The generic
recipe for almost any ARM flash controller:

1. **Unlock** — write a magic key sequence to the KEY register (PY32: `0x45670123` then
   `0xCDEF89AB` to `FLASH_KEYR`). Reading `FLASH_CR` back showed the LOCK bit clear →
   unlocked.
2. **Erase** — set the "page erase" bit, then write once to any address in the target
   page to trigger it. Poll the BSY bit until done. Verify the page reads `0xFF`.
3. **Program** — set the "program" bit, write the words, poll BSY.

The single most important enabler: **the running firmware had already configured the
flash timing registers** (`0x40022100…`). Those are the fiddly, frequency-dependent part
of PY32 flashing. By attaching with `halt` (which *pauses* the CPU but keeps its state)
instead of `reset halt` (which wipes it), those registers stayed valid and I could reuse
them. **`halt` vs `reset halt` is a critical distinction** — one preserves runtime state,
the other gives you a pristine-but-empty machine.

Safety technique used: I validated the whole erase+program path with a **non-destructive
round-trip** — erase one page, verify `0xFF`, write the *original* bytes back, verify
identical. That proves the write path works while leaving the chip unchanged. Only after
that did I do anything for real. **Always prove a destructive capability on recoverable
data first.**

---

## 4. Phase 3 — Understanding the protocol (cheap, do it early)

Before disassembling anything, I captured the real UART traffic with pyserial: send the
query command, read the 11-byte response, print hex, repeat while conditions change.

```
FE FE 68 | 01 00 | 01 60 | 02 F8 | F2 | 16
          (b3,4)  (b5,6)  (b7,8) cksum
          =256    =352    =live
```

By watching which bytes moved and which were frozen, I localized the problem to *specific
bytes* (b5,6 = frozen TVOC; b7,8 = live raw ADC) **without touching the firmware at all.**
This is the cheapest, highest-leverage step and beginners routinely skip it. Observe the
black box before you open it.

---

## 5. Phase 4 — Static analysis: finding the needle

### 5.1 The anchor technique (the core skill)
30 KB of Thumb code is far too much to read. The winning move is to search for **anchors**
— addresses the code *must* reference. Cortex-M loads constants and peripheral addresses
from **literal pools** (small data tables between functions), as raw 32-bit
little-endian words. So I scanned the raw binary for the LE encodings of things I cared
about:

- Peripheral base addresses (ADC `0x40012400`, USART `0x40013800`, FLASH `0x40022000`…)
- The flash calibration page address (`0x08008F80`)

```python
import struct
def find(word):
    t = struct.pack("<I", word)
    i, out = 0, []
    while (j := data.find(t, i)) >= 0:
        out.append(0x08000000 + j); i = j + 1
    return out
```

Every hit is a literal-pool entry, and the code that uses it sits just above (a
PC-relative `ldr`). This instantly located the ADC-read code, the UART code, and the
calibration logic — turning "read 30 KB" into "read three 200-byte windows."

### 5.2 DEAD END #1 — objdump mangles data
My first disassembly attempt used `objdump -D -b binary -m arm -M force-thumb`. It
"worked" but found **zero** literal-pool references. Why? `objdump -D` disassembles
*everything*, including the literal-pool data words, as if they were instructions —
so the `.word 0x40012400` I was grepping for got decoded as two bogus Thumb ops and never
appeared as text. **Takeaway:** a linear disassembler can't tell code from data. Either
scan the raw bytes (§5.1) or use a tool that does control-flow analysis (Ghidra). I
switched to scanning raw bytes + Capstone on *specific* ranges.

### 5.3 Capstone with literal resolution
The key upgrade over plain disassembly: for each `ldr rX, [pc, #imm]`, compute the
literal address and **read the actual value from the image**, then annotate the line.
Suddenly the disassembly reads like: "load 4095.0", "load ADC base", "load 3.3" — the
constants tell you what the code is doing.

```python
if ins.mnemonic.startswith("ldr") and "pc" in ins.op_str:
    lit = ((ins.address + 4) & ~3) + imm     # Thumb PC = addr+4, word-aligned
    value = read_word(lit)                    # <-- the magic: show the constant
```

### 5.4 Identifying the soft-float library by sight
Cortex-M0 has **no hardware floating point**, so all float math is function calls into
the compiler runtime (`__aeabi_fadd`, `fmul`, `fdiv`, `f2d`, `d2f`, …). These have
recognizable fingerprints — e.g. `fsub` is literally "flip the sign bit of the second
operand, then jump to `fadd`":
```
movs r2, #1; lsls r2, r2, #0x1f;  eors r1, r2;  b <fadd>   ← this is __aeabi_fsub
```
Once you identify these ~10 helpers, a wall of `bl 0x800xxxx` calls turns into readable
arithmetic: `fdiv(x, 4095)`, `fmul(_, 3.3)`, etc. **Recognizing library routines is a
force-multiplier** — learn the common ones.

---

## 6. Phase 5 — Dynamic analysis: let the chip do the work

This is where it got fast. Instead of tracing the whole math chain by hand, I used the
live debugger to read the actual numbers.

### 6.1 Finding a variable by its value (float bit-patterns)
The stuck TVOC was 0.352 ppm. Floats have a fixed bit layout, so I dumped RAM and searched
it for the IEEE-754 encoding of ~0.352:
```python
for i in range(0, len(ram), 4):
    f = struct.unpack_from("<f", ram, i)[0]
    if 0.30 < f < 0.40:
        print(hex(0x20000000+i), f)     # found 0.35253 at 0x20000360
```
That located the TVOC variable's RAM address directly. (Same trick finds the live frame
buffer, calibration params, etc. Learn to recognize float ranges in hex: `0x3E–0x3F`
exponents ≈ values around 0.1–1.0.)

### 6.2 The watchpoint — catching the code red-handed
Knowing *where* the value lived, I set a **hardware watchpoint** on writes to that address
and let the firmware run. The CPU halts the instant anything writes it:
```
wp 0x20000360 4 w     # break on write to the TVOC float
resume
wait_halt 10000
```
It halted with the PC sitting right at the store instruction, inside an ADC interrupt
handler. **This is the single most powerful RE technique for "who sets this value?"** —
it replaces hours of backward dataflow tracing with one breakpoint. Cortex-M0 gives you
~2 watchpoints; spend them wisely.

### 6.3 Reading the inputs live
With the computation located, I read the live values of its inputs (the resistance `rs`,
the baseline `R0`) straight out of RAM instead of deriving them. Then I sampled them in a
loop while the sensor ran, which revealed the actual behavior: TVOC tracked the slow
baseline `R0` and ignored the live signal — the root cause. **Static analysis found the
formula; dynamic analysis found the *bug*.** You usually need both.

### 6.4 Closing the loop — numerical validation
Final step, and a non-negotiable one: I plugged the live `R0` into the reconstructed
formula in Python and checked it reproduced the observed output **bit-exactly**:
```
TVOC = 11.5 - 1.86*ln(501 - R0)
R0=100.28  ->  0.35253   ==   observed 0.35253 (0x3EB47F0C)  ✓
```
If your reconstruction doesn't reproduce reality to the last digit, you've mis-identified
an operator or constant. **Always validate against ground truth** — it's how you know
you're *right*, not just plausible. (This is also how I confirmed the log was natural log,
not log10: log10 gave 6.66, ln gave 0.35253.)

---

## 7. DEAD END #2 — the red herring that taught the real lesson

Early on, the dump showed a lone 128-byte data page isolated after a gap — textbook
"calibration store." I erased it expecting the sensor to recalibrate and unfreeze. It read
0.000 briefly… then re-converged to 0.352. **Wrong theory.**

But this "failure" was actually decisive evidence: it *proved* the stuck value is not
stored state — the firmware *recomputes* it from the current (drifted) sensor condition
every boot. Later, the firmware even rewrote that page back to the identical original
bytes on its own. **A disproven hypothesis that changes your understanding is a success,
not a waste.** Design experiments to be informative whether they pass or fail.

---

## 8. What I'd do differently / how to go faster

- **Use Ghidra (free) or IDA.** Load the dump at base `0x08000000`, CPU = Cortex Thumb,
  import the PY32F003 SVD for peripheral names, and mark the `__aeabi_*` helpers. You get
  **decompiled C-like pseudocode automatically** and proper code/data separation — no
  literal-pool grepping, no hand-tracing soft-float. I went the Capstone route because I
  was over SSH targeting one equation; for breadth, a decompiler wins outright.
- **Signature-match the vendor SDK.** Puya's PY32F0 SDK is open source. Compile it and use
  FLIRT-style matching to auto-tag every library function in the dump; whatever's left
  unmatched is the application code. This *partitions* the problem for free.
- **Script your disassembler once, reuse forever.** The annotated-Capstone helper (resolve
  literals, name known call targets) paid for itself many times over.

---

## 9. A reusable checklist

```
[ ] Identify the MCU (markings, then confirm via debug ID registers)
[ ] Check target logic voltage BEFORE wiring anything
[ ] Connect over SWD/JTAG; confirm core detected (DPIDR / IDCODE)
[ ] Dump flash; validate vector table (SP->RAM, reset->flash) = RDP off + real image
[ ] Dump twice, compare hashes (reader must be deterministic)
[ ] Observe external behavior (UART/protocol) before opening the binary
[ ] Scan raw binary for anchors (peripheral bases, key addresses, constants)
[ ] Disassemble ONLY the anchored windows; resolve literals; name library calls
[ ] If you can run it: find variables by value, set watchpoints, sample live
[ ] Reconstruct the logic; VALIDATE numerically against ground truth
[ ] Keep a lab notebook of hypotheses — including the ones you disprove
```

---

## 10. Glossary

- **SWD** — Serial Wire Debug, ARM's 2-pin debug protocol (SWDIO + SWCLK).
- **DPIDR / IDCODE** — ID register read at debug-connect; confirms the link works.
- **RDP** — Readout Protection; locks flash from being dumped. Clearing it usually erases
  the chip.
- **Vector table** — table at the start of flash; entry 0 = initial SP, entry 1 = reset
  handler. Your sanity check for "is this a real image?"
- **Literal pool** — small blocks of constant *data* embedded in *code*; how Cortex-M
  loads 32-bit constants and addresses. Both a treasure map (anchors) and a trap (linear
  disassemblers decode them as instructions).
- **Soft-float / `__aeabi_*`** — compiler runtime routines that emulate float/double math
  on CPUs without an FPU (all Cortex-M0).
- **Watchpoint** — hardware breakpoint that fires on memory *access* to an address; the
  fastest way to answer "what writes this variable?"
- **`halt` vs `reset halt`** — pause the running CPU (state preserved) vs reset then pause
  (state wiped). Choose deliberately.

---

*Companion files in this folder: `backup.bin` (the firmware image), `py32_swd.cfg` (the
OpenOCD SWD config), `tvoc_live.py` (the host-side fix), `TVOC_report.md` (the findings /
Waveshare report).*
