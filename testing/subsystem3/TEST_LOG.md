# Buddy 3 Line barcode and junction perception test record

This is a proposed plan, not test evidence. No hardware test was run during
folder migration. Define exact preconditions, thresholds, procedures and evidence
before execution. Electrical work requires G1; active motion needs explicit
bounded-test approval and a physical power cut.

## Proposed cases

| ID | Purpose | Method and acceptance | Status |
|---|---|---|---|
| S3-U10 | Measure black/white contrast at actual mounting height | Procedure and quantitative acceptance to be agreed | Not run |
| S3-U11 | Validate Code 39 decoding and junction evidence independently | Procedure and quantitative acceptance to be agreed | Not run |

The code's subsystem3_test() is only a SKIP placeholder and is not invoked by
the inactive app_main. Do not report it as a subsystem PASS.

## Results

Append actual runs only. Keep failures and link the exact tested commit, profile,
configuration and raw log/video. Upstream example rows are archived separately
under docs/template-reference and are not results for this team.

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
