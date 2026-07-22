# Contributing

How changes get into `main`: issues, branches, pull requests, reviews, and CI.

## Issues

- Work is tracked in GitHub issues - one issue per bug, feature, or improvement - so nothing lives only in someone's head or a chat scroll.
- A PR that finishes an issue closes it via the PR template ("Closes #NN"); partial progress gets a comment on the issue instead.
- Assign yourself to an issue before starting on it, so two people don't build the same thing.

## Branches

- Work happens on `uniqname/kebab-description` branches (e.g. `ryanliao/doc-fixes`) branched from `main`.
- At the field, iteration speed beats clean history. On a test day, dump changes onto a shared `test-<date>` branch (e.g. `test-07-18`) instead of committing to `main`. `comp` is the equivalent shared branch for competition.
- Afterwards, sort the test branch's changes into normal PRs and land them in `main` as soon as possible, ideally before the next test day so the two never drift apart.

## Pull Requests

Every change lands through a PR, squash-merged into `main`. **The PR title becomes the commit on `main`**. Write it like a changelog entry ("Move all yaml config in bringup into the respective nodes"), not a work log ("more stuff").

For a feature split across multiple PRs, use the stacked-PR title convention so the pieces read together in history:

```
(1/4 Autonav mission control) Add MissionState message
(2/4 Autonav mission control) Implement mission control node
```

Fill in the PR template: the issue it closes, what changed, and how it was tested.

The author merges their own PR once it's approved and CI is green, and an approved PR shouldn't sit unmerged.

## Reviews

Request a review from the lead or assistant lead of the subteam that owns the code you're changing. Leads are free to delegate the review to other members of their subteam, but every affected subteam must sign off with at least one review.

## CI

Every PR must pass the lint and test workflows, which run `just lint`, `just build`, and `just test` on Linux (x64 and arm64) and macOS runners. The entire toolchain is pinned by `pixi.lock`, so local runs of the same commands reproduce CI exactly.
