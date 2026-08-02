# Contributing

How changes get into `main`: [issues](#issues), [branches](#branches), [pull requests](#pull-requests), [reviews](#reviews), and [CI](#ci).

> [!WARNING]
> This document assumes basic knowledge of Git and GitHub. If you find yourself confused at some of the terminology, feel free to pause and look up definitions. If you still find yourself lost, reach out to your subteam lead.

## Issues

Issues make sure progress and ideas are visible.

- Work is tracked in GitHub issues, one issue per bug, feature, or improvement
- A PR that finishes an issue closes it via the PR template
- If your progress isn't visible from a linked PR yet, comment updates on the issue
- Assign yourself to an issue before starting on it

## Branches

Work happens on [`uniqname/kebab-description`](https://developer.mozilla.org/en-US/docs/Glossary/Kebab_case) branches (e.g. `ryanliao/doc-fixes`) branched from `main`.

The exception to this rule is during integration testing or competitions:

- On a test day, dump changes onto a shared `test-<date>`, for testing, or `comp`, for competitions, branch instead of committing to `main`.
- Afterwards, make the test branch's changes into normal PRs and land them in `main` as soon as possible, ideally within the same day to prevent drifting too far from `main`.

## Pull Requests

Every change lands through a PR. It's important you title the PR clearly, as its title will become the description of the commit generated when it squash-merges.

- Good:
  - "Move all yaml config in bringup into the respective nodes"
  - "Improve path tracing accuracy"
- Bad:
  - "more stuff"
  - "change path tracing"

For a feature split across multiple PRs, use the stacked-PR title convention so the pieces read together in history:

```
(1/4 Autonav mission control) Add MissionState message
(2/4 Autonav mission control) Implement mission control node
```

Fill in the PR template with the issue it closes, what changed, and how it was tested.

The author merges their own PR once it's approved ([Reviews](#reviews)) and CI is green ([CI](#ci)). Once your PR is approved, you should merge it ASAP to prevent merge conflicts and clutter.

## Reviews

Request a review from the lead or assistant lead of the subteam that owns the code you're changing. Leads are free to delegate the review to other members of their subteam, but every affected subteam must sign off with at least one review.

## CI

Every PR must pass the lint and test workflows, which run `just lint`, `just build`, and `just test` on Linux (x64 and arm64) and macOS.
