# Results

`results.csv` is an append-only log of every number this project reports. It is written only by:

```
uv run --project sim python -m daruma.results add --metric <hw|sim>.<area>.<name> --value <v> --unit <u> \
    --method "<how it was measured>" --evidence <raw file or manual:<instrument>>
```

| Column | Meaning |
| --- | --- |
| `date` | when the row was recorded |
| `metric` | `hw.*` for the real robot, `sim.*` for simulation. They are never mixed in one claim. |
| `value`, `unit` | the number, as measured, not rounded |
| `method` | how it was measured |
| `evidence` | the raw file in `raw/` (serial log, logic-analyzer export, CSV), or `manual:` for an instrument reading |
| `commit` | the code version that produced it (`+dirty` if uncommitted changes existed) |
| `note` | corrections refer to the earlier row here; rows are never edited or deleted |

No results yet.
