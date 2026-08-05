# §0 · Working rules

**Read this before touching any file.** These rules are ordered by priority; when two conflict,
the lower number wins.

---

1. **One file per change.** Implement a file completely, build it, then move on. Do not open six
   files and leave all of them half-finished.

2. **Do not change public signatures.** The headers in `uav_mpc/include/uav_mpc/` and the
   `.msg` / `.srv` / `.action` definitions are the contract between subsystems. If a signature
   is genuinely wrong, **stop and say so** instead of quietly editing it — several files depend
   on each one, and a silent signature change breaks them in ways that surface much later.

3. **Delete the `TODO(deepseek)` comment when you implement it.** A leftover TODO on implemented
   code is a lie that costs the next reader ten minutes. `format_check.yml` counts the remaining
   markers as a progress meter.

4. **Never invent a physical constant.** If you do not have a source for a number, leave the
   placeholder and mark it `# UNVERIFIED`. A fabricated inertia tensor discredits the whole
   repository — and this repository's entire value is that its numbers can be trusted.

5. **Every claim needs a check.** If you implement something whose correctness is not obvious
   (a Jacobian, a frame conversion, an allocation matrix, a flatness map), add the test that
   proves it **in the same change**. See [10_TESTS.md](10_TESTS.md).

6. **Build order matters.** Follow the milestones in [15_ROADMAP.md](15_ROADMAP.md). The
   dependency chain is real: nothing above a layer compiles until the layer below does.

7. **If a step is blocked** — missing acados, missing PX4, an unresolved version question, a
   decision these documents do not settle — implement everything that is *not* blocked, then
   report exactly what is blocked and why. Do not stub around it silently and do not guess.

8. **Report honestly.** If a test fails, say so and show the output. If you skipped something,
   say that. A green summary over a broken build is the most expensive thing you can produce
   here, because it will be believed.

---

## What "implemented" means

A file is not done when it compiles. It is done when:

- every `TODO(deepseek)` in it is implemented and the marker deleted, or converted into a
  specific written issue with a reason
- it builds with `-Wall -Wextra -Wpedantic -Wconversion` clean (the warnings are on
  deliberately — fix them, do not silence them)
- its tests pass, in a Release build
- any behaviour a reader would not predict from the signature is documented in a comment

The full definition is §17 in [15_ROADMAP.md](15_ROADMAP.md).

## When a document is wrong

These documents were written before the code existed. Some of it will turn out to be wrong —
particularly the version pins in [02_ENVIRONMENT.md](02_ENVIRONMENT.md) and the physical
constants in `uav_mpc/params/`.

When you find an error: **fix the document in the same commit as the code.** A specification
that has silently drifted from the implementation is worse than no specification, because the
next reader will trust it.

Do not weaken an acceptance criterion or a test threshold in place. If a criterion cannot be
met, change it explicitly in [01_OVERVIEW.md](01_OVERVIEW.md) with a written reason.
