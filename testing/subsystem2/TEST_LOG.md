# Buddy 2 Motion test record

This is a proposed plan, not test evidence. No hardware test was run during
folder migration. Define exact preconditions, thresholds, procedures and evidence
before execution. Electrical work requires G1; active motion needs explicit
bounded-test approval and a physical power cut.

## Proposed cases

| ID | Purpose | Method and acceptance | Status |
|---|---|---|---|
| S2-U10 | Verify wiring and signal levels before power tests | Procedure and quantitative acceptance to be agreed | Not run |
| S2-U11 | Measure encoder counts separately per wheel and direction | Procedure and quantitative acceptance to be agreed | Not run |

The code's subsystem2_test() is only a SKIP placeholder and is not invoked by
the inactive app_main. Do not report it as a subsystem PASS.

## Results

Append actual runs only. Keep failures and link the exact tested commit, profile,
configuration and raw log/video. Upstream example rows are archived separately
under docs/template-reference and are not results for this team.

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-21 | S2-U11 | Uncommitted working tree; `TEST=1 CONSOLE=usb_cdc`; diagnostic UF2 SHA-256 `17EB8EEF58BE75005C8837418B1F3D0BA5F2783277CE7E14FD4BCD58AEDA13A6` | Team bench run (name not recorded) | INCONCLUSIVE (initially recorded FAIL) | [`logs/2026-10-21_encoder-zero-transitions.txt`](logs/2026-10-21_encoder-zero-transitions.txt) | Right GP5 and alternate GP4: 0 transitions in 1319 ms. Left GP27: 0 transitions in 979 ms. Later review found this image sampled via a 10 ms kernel tick, too slowly for the motor encoder, so it cannot establish a hardware failure. |
| 2026-10-21 | S2-U11 repeat | Uncommitted working tree; `TEST=1 CONSOLE=usb_cdc`; protected-sonar diagnostic UF2 SHA-256 `C5D8843CCB4BEF6FD93B3E90CC3B3B6EA8CAF382B25645DE23754DAFBC44796D` | Team bench run (name not recorded) | INCONCLUSIVE (initially recorded FAIL) | [`logs/2026-10-21_encoder-zero-transitions-repeat.txt`](logs/2026-10-21_encoder-zero-transitions-repeat.txt) | Right GP5 and alternate GP4: 0 transitions in 1559 ms. Left GP27: 0 transitions in both 1039 ms and 1219 ms repeats. Later review found this image sampled via a 10 ms kernel tick, so the repeated zero result still cannot establish a hardware failure. |
| 2026-10-21 | S2-U11 fast-sampler repeat | Uncommitted working tree; `TEST=1 CONSOLE=usb_cdc`; diagnostic UF2 SHA-256 `D4154C4EDC5E7E4D6FA037A10FA74509BBB2FBF8BEE271805A1218F09251E6B6` | Team bench run (name not recorded) | FAIL | [`logs/2026-10-21_encoder-fast-sampling-zero-transitions.txt`](logs/2026-10-21_encoder-fast-sampling-zero-transitions.txt) | Corrected 50 us sampling: right GP5 remained 0 and left GP27 remained 1 while their motors ran; both reported 0 transitions. The encoder power/signal/harness path requires electrical diagnosis before calibration. |
| 2026-10-21 | S2-U11 dual-channel pull probe | Uncommitted working tree; `TEST=1 CONSOLE=usb_cdc`; diagnostic UF2 SHA-256 `CAD08872B95FE6964F23D55FA2DA28AD077668C91648399CE96AFB662345606B` | Team bench run (name not recorded) | FAIL | [`logs/2026-10-21_encoder-dual-channel-pull-probe.txt`](logs/2026-10-21_encoder-dual-channel-pull-probe.txt) | Right GP5/GP4 were held low and produced only 2/1 transitions. Left GP27 floated with 0 transitions; GP26 was held low with 8 transitions. Inspecting the motor-specific positional pinout indicates the earlier generic colour mapping likely placed VCC and an output on the wrong conductors. |
| 2026-10-21 | S2-U11 corrected pinout | Uncommitted working tree; `TEST=1 CONSOLE=usb_cdc`; diagnostic UF2 SHA-256 `8067B0D05A7F42E0FB15B774E939EB8D06A52D4A157911E2BFF8B3C3C64395A7` | Team bench run (name not recorded) | PASS (signal bring-up; calibration pending) | [`logs/2026-10-21_encoder-corrected-pinout-pass.txt`](logs/2026-10-21_encoder-corrected-pinout-pass.txt) | Corrected blue=GND, red=A, green=B, white=3V3 mapping produced right B/A=1206/1205 and left B/A=1238/1238 transitions over approximately one wheel revolution. Both encoders and both quadrature channels are functional; use a multi-revolution measurement for final counts/revolution. |

The encoder diagnostic was corrected after these runs to sample from the
hardware microsecond timer every 50 microseconds. Retest both encoders with a
new UF2 before changing the confirmed motor/encoder wiring or replacing parts.

## 2026-10-28 B2-01 software foundation

User authorized implementation and software tests only; no flash, powered motion,
servo action or wiring change. Drivers, lifecycle and RP2040 BSP adapter added in
subsystem2; normal app_main remains inactive and an explicit reviewed platform
binding is required before init/start can operate.

- New isolated fake-HAL CTest suite: 28/28 PASS, GCC 15.2.0, C11,
  warnings-as-errors, assertions enabled. Includes init/active-output/STOP
  failures, persistent failure reporting, STOP interleavings, independent
  encoder capture, reset/wrap, stale authority and real lifecycle rollback.
- Existing shared host suite: 4/4 PASS; no shared test/build/contracts edited.
- Five foundation sources compile with real course headers for Cortex-M0+
  using ARM GCC 15.2.1 and warnings-as-errors. Full course image also links in
  an isolated snapshot, SMP=1/USB CDC/UART0-free/WIFI=none/TEST=0.
- Exact commands, evidence boundaries and later hardware prerequisites:
  [`b2_01/README.md`](b2_01/README.md). Logs/manifests are in workspace
  `output/planning/2026-10-28-integration-review/buddy2-b2-01/`.

These are software results. The shared target lock/dispatcher binding, physical
disabled state, <=50 ms STOP, edge-loss envelope and loaded acceptance remain
pending. The emitted inactive-build UF2 is not a hardware test image.

## 2026-10-28 B2 runtime binding and bounded bench image

User authorized the next runtime/software-build slice; the prior no-flash/no-
movement restriction remains. Added shared GPIO dispatcher, core-0 SMP lock /
clock binding, first-edge physical STOP, alarm0 cutoff, task-context controller
reconciliation and a core-0-pinned bench task with 4096-byte static stack.

- 38/38 B2 CTest cases PASS, including real platform code with MMIO/IRQ/BSP/
  kernel-lock mocks; task create/start rollback, boot-held/bounce/expired manual
  gestures, STOP races, simultaneous encoder/sonar dispatch and conflict/clock
  refusals. All four existing host regressions PASS.
- Strict ARM course-header compilation PASS. Dedicated B2_BENCH=1 course build
  PASS; normal B2_BENCH=0 build also verified separately. No source branch,
  controller/protocol enum, kernel tick or fault-bit allocation changed.
- Distinct image and hashes: workspace output/firmware/buddy2-runtime-2026-10-28/.
  Procedure and limitations: [B2_RUNTIME_TEST_GUIDE.md](B2_RUNTIME_TEST_GUIDE.md).
- No powered test, board flash, physical STOP latency, edge-loss measurement,
  floor/load/PID/distance/pivot or ramp qualification was performed. Software
  shutdown-call timing is not a physical <=50 ms measurement.
