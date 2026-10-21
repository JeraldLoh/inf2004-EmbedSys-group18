# Shared contracts

Existing portable types and message layouts were preserved. All buddies review
changes to units, validity, command/goal IDs, generations and expiry semantics.
These C structures are not yet mapped to allocated micro T-Kernel objects.

Finite goal timestamping, freshness rules for sensor evidence, recovery authority,
network transport and all interface ownership still need the detailed review in
`../../docs/interfaces/contracts.md` and `../../TEAM.md` before runtime use.
