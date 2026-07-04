# Waveshare TVOC Sensor — Diagnostic Report

**Module:** Waveshare TVOC Sensor (MOX element + Puya **PY32F003F18P6** MCU, UART interface)
**Symptom:** Computed TVOC output frozen at **~0.352 ppm (352 ppb)** regardless of air quality.
**Date of investigation:** 2026-07-04
**Tools:** Raspberry Pi Zero W as bit-banged SWD adapter (OpenOCD 0.12), firmware dump + reverse engineering (capstone), live RAM inspection via hardware watchpoints.

---

## 1. Summary

The sensor's reported TVOC value is stuck at ~0.352 ppm and never moves, even though the
raw ADC signal still responds strongly to VOCs. Root cause was traced by dumping and
reverse-engineering the firmware and inspecting the live target over SWD.

**Conclusion:** This is **not** a corrupted-flash or protocol problem. The firmware's TVOC
computation depends only on a heavily-smoothed internal baseline-resistance value (`R0`),
which has drifted after ~1 year of unpowered storage to an operating point that pins the
output near 0.352 ppm. Erasing/reflashing cannot fix it (the value re-converges to the
same point). The **raw ADC signal is healthy and responsive**, so a usable TVOC reading
can be recovered host-side.

---

## 2. Evidence the hardware (MOX + ADC) is alive

Raw ADC (exposed in the query-response frame, bytes 7–8) responds normally:

| Condition        | Raw ADC (bytes 7,8) |
|------------------|---------------------|
| Clean air        | ~708–760 (and ~848 after longer warm-up) |
| Under alcohol vapour | drops to ~247       |

A ~3× swing under VOC proves the MOX element and the MCU ADC work correctly.
Meanwhile the computed TVOC field (bytes 5–6) stays frozen at 352 (=0x0160).

Raw query frame observed (query command `FE 00 86 00 00 00 00 00 86`):

```
FE FE 68 | 01 00 | 01 60 | 02 F8 | F2 | 16
          (b3,4)  (b5,6)  (b7,8) cksum
          =256    =352    =live ADC
                  FROZEN  RESPONSIVE
```

---

## 3. Root cause — firmware TVOC formula (reverse-engineered)

The TVOC value is computed inside an ADC/timer interrupt handler (flash `0x08002120`–`0x0800224C`)
and stored as a float at RAM `0x20000360`. The final transform, decoded from the machine code and
**validated bit-exactly against the live device**, is:

> **TVOC(ppm) = 11.5 − 1.86 × ln(501 − R0)**

where `R0` is an internal, heavily rank/percentile-filtered **baseline-resistance** value stored at
RAM `0x200004CC` (`ln` = natural logarithm).

Verification against the live device:

| Live `R0` (read via SWD) | Formula output | Device output |
|--------------------------|----------------|---------------|
| 100.280                  | 0.35253 ppm    | 0.35253 ppm (0x3EB47F0C) ✓ |
| 99.97                    | 0.351 ppm      | 0.351 ppm ✓   |

### Why it is stuck
- TVOC depends **only** on the smoothed `R0`, not on the fast live signal.
- Sensitivity is `d(TVOC)/d(R0) = 1.86 / (501 − R0) ≈ 0.0046 ppm per unit`.
- After long unpowered storage the MOX baseline drifted so `R0` parks at **~100.3**, which pins
  TVOC at ~0.352 ppm. On any power-up / recalibration, `R0` re-converges to ~100 and the output
  returns to 0.352. The heavy rank-filter on `R0` also compresses the live signal, so even large
  raw-ADC changes barely move `R0`.

This is consistent with known MOX behaviour: sensors drift significantly after long unpowered
storage and typically require extended continuous burn-in to recover.

---

## 4. What was ruled out

- **Read protection (RDP):** OFF (`FLASH_OPTR` low byte = 0xAA). Full SWD read/write confirmed.
- **Corrupted stored calibration:** The lone flash calibration page (`0x08008F80`, 24 bytes of
  data) was erased; the firmware re-initialised and re-converged to 0.352. So the stuck value is
  **not** persisted state — it is produced fresh by the algorithm each run.
- **UART command to reset/recalibrate:** A full command-byte sweep (0x00–0xFF) found only two
  implemented commands (0x86 read, 0x78 mode-switch). No reset/calibration command exists.
- **Reflashing the same firmware:** Would not help — the algorithm itself produces 0.352 from the
  current sensor state.

---

## 5. Recovered working reading (host-side)

Because the raw ADC (frame bytes 7–8) is fully responsive, a live TVOC can be computed on the host,
bypassing the firmware's over-smoothed baseline. Script: `~/tvoc_live.py`

```
TVOC = 1.86 * ln(ADC0 / ADC)      # ADC0 = self-calibrated clean-air baseline (cleanest ADC seen)
```

| Raw ADC | Condition     | Host-computed TVOC |
|--------:|---------------|--------------------|
| 760/848 | clean air     | 0.000 ppm          |
| 620     | mild VOC      | 0.38 ppm           |
| 400     | moderate      | 1.19 ppm           |
| 247     | alcohol wipe  | 2.09 ppm (alarm)   |

Run: `python3 ~/tvoc_live.py` (Ctrl-C to stop) or `python3 ~/tvoc_live.py 15` for a fixed count.

**Note:** absolute ppm is uncalibrated (relative index). One known-concentration reference gas point
would let the `1.86` coefficient be fixed for true ppm; the shape/responsiveness is already correct.

---

## 6. Files

| File | Purpose |
|------|---------|
| `~/backup.bin` | Full 64 KB firmware dump (md5 `dfc486460274d4b1ca0e1c057796c5de`) |
| `~/tvoc_live.py` | Host-side responsive TVOC from raw ADC |
| `~/TVOC_report.md` | This report |
