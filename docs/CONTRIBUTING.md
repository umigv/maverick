# Contributing

How changes get into `main`: branches, pull requests, CI, and the documentation conventions. For environment setup and the build/test loop, see [DEVELOPMENT.md](DEVELOPMENT.md).

## Branches

- Work happens on `username/kebab-description` branches (e.g. `ryanliao/doc-fixes`) cut from `main`.
- `comp` is the shared competition branch for changes made at the field.

## Pull Requests

Every change lands through a PR, squash-merged into `main`. **The PR title becomes the commit on `main`** - write it like a changelog entry ("Move all yaml config in bringup into the respective nodes"), not a work log ("more stuff").

For a feature split across multiple PRs, use the stacked-PR title convention so the pieces read together in history:

```
(1/4 Autonav mission control) Add MissionState message
(2/4 Autonav mission control) Implement mission control node
```

Fill in the PR template: the issue it closes, what changed, and how it was tested.

## CI

Every PR must pass the lint and test workflows, which run `just lint`, `just build`, and `just test` on Linux (x64 and arm64) and macOS runners. The entire toolchain is pinned by `pixi.lock`, so local runs of the same commands reproduce CI exactly.

## Documentation

- Every package has a README documenting its interface and behavior; `just create-package` scaffolds one. Update it in the same PR as the behavior it documents.
- Section vocabulary (in this order, behavior sections first): free-form behavior/algorithm sections, then `Subscribed Topics`, `Published Topics`, `Services`, `Service Clients`, `Read Files`, `Written Files`, `TF Broadcasts`, `TF Requirements`, `Config Parameters`, `Scripts`. Delete sections that don't apply.
- Parameters: Python packages document them in the config dataclass, not the README. C++ packages (no config loader) get a `Config Parameters` table.
- Markdown is soft-wrapped: one line per paragraph, no manual line breaks. `just format` enforces this and the rest of the markdown style via mdformat (configured in `pyproject.toml`); `.editorconfig` and the VS Code word-wrap settings keep soft-wrapped lines readable in editors.
- Use a plain hyphen surrounded by spaces as the separator, never an em-dash. Bullet descriptions after the separator start capitalized.
- Topic bullets follow `` `topic` (`pkg/Msg`) - Description `` with the `msg`/`srv` segment omitted from type names.
