# Buddy 3 saved test evidence

Save raw serial or wireless log extracts from Buddy 3 tests here. No logs are
stored yet; the on-car runs so far (E1 to E4) are summarised in
[`../../DEMO_EVIDENCE.md`](../../DEMO_EVIDENCE.md).

Name each file `YYYY-MM-DD_<test-id>_<short-note>.txt` (for example
`2026-11-26_S3-T03_letter-A.txt`). At the top of the file, note the test ID,
date, commit, build command or profile, sensor height, lighting and units.
Link each run from the Results table in [`../TEST_LOG.md`](../TEST_LOG.md),
including failed runs.

Useful lines to keep:

- `[S3 CAP]` for every barcode capture. Replay them with
  `python tools/barcode_replay.py <log> --expect <letter>`.
- `[S3] Route ...`, `[S3] Barcode NOT decoded ...` and `[S3 RETRY]` for
  barcode results.
- `[GROUND]` (with `S3_GROUND_LOG=1`) for line and barcode sensor readings.

Do not put credentials in logs. Videos and photos belong outside Git; add
their links instead.
