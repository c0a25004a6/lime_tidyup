# dev2 provenance history repair — 2026-08-25

Status: COMPLETED
Scope: `dev2` only. `main`, `dev`, `arm`, `voice`, and other pre-existing branches were not rewritten.

## Source boundary

- Repository: `nekomario28/lime_tidyup`
- Canonical source: `momoiorg-repository/LimeSimulDemo`
- Canonical baseline: `737274733d018596cef24e5d9f9c7c6e4834159d`
- Baseline Author: Kazuya Tago `<ktago.gm@gmail.com>`
- Old standalone root: `5252d3a1fd2c5d987a414772f2caf010964b4380`
- Old dev2: `66c20a90a2f0a1652716f53df886ac9603fbebb1`
- New root: `8363ae8adcef28c19d2c0370b355bb2953a51647`
- New dev2: `22c9578627a5e552352c86071c7a5bd6d6c52a20`
- Content tree before/after: `d9fcc2aa0a6c6fd28e4d25f412790064b0649534`
- Exact old backup: `backup/pre-provenance-dev2-66c20a90`

The old standalone root reused the exact canonical README blob from the baseline. The repair grafted that local root onto the canonical baseline, then recreated all 72 local commits with their exact old trees, Author/AuthorDate, Committer/CommitterDate, messages, and merge topology; only parent object IDs changed.

## Validation evidence

- Read-only preview run: `32832980025` — SUCCESS
- Candidate staging run: `32833146806` — SUCCESS
- Coordinated rewrite run: `32833519985` — SUCCESS
- 72/72 local commits mapped old→new
- final tree equality: PASS
- `git diff old_dev2 new_dev2`: empty
- open PRs touching `dev2`: 0 at execution
- tags containing the old root/dev2: none at execution
- releases: 0 at execution
- update method: `git push --force-with-lease=refs/heads/dev2:<old_sha>`

## Line-level provenance after repair

Representative `git blame -w -M -C -C` results from the verified candidate:

- `README.md`: 96 lines Kazuya Tago
- `Dockerfile`: 73 Kazuya Tago; 23 Yuu Akaoka; 20 c0a2504494; 2 yulat214
- `pytwb_ws/src/cm1/cm1/behavior/setlocations.py`: 34 Kazuya Tago; 8 Yuu Akaoka; 7 Yuu
- `pytwb_ws/src/cm1/cm1/lib/actor/cognitive.py`: 245 Kazuya Tago; 229 Yuu Akaoka; 11 c0a2504494

This demonstrates why canonical-ref reachability and line-level blame are stronger provenance evidence than merely resolving a SHA through a GitHub fork network.

## Mapping

The complete 72-entry old→new SHA map is stored in `docs/provenance-dev2-old-to-new.tsv` beside this receipt.
