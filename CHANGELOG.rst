^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uav_mpc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-08-06)
------------------
* Initial release scaffolding.
* Implementation of milestones M1-M10 per the design specification:
  frames and dynamics, symbolic model, solver generation, trajectories,
  solver wrapper, lifecycle node, launch files, CI workflows, analysis and
  media scaffolding, and documentation.
* Decouple the controller from ``px4_msgs`` behind a runtime vehicle backend
  (``generic`` by default, ``px4`` optional), required because ``px4_msgs`` is
  not released for ROS 2 Lyrical Luth.
* Integrate acados 0.6.0 with a CasADi 3.7.2-generated SQP-RTI solver
  (content-hashed codegen output, committed model hash).
* Round 1 review fixes (the review rounds): solve-time budgets from
  ``acceptance_criteria.yaml``, hover test gating, CI lint paths, frame
  conversion edge cases, and more; all verified by the test suite.
* Round 2 review fixes: yamllint targets
  real directories, RViz config is no longer a placeholder, CITATION.cff
  added, requirements pins marked verified, release scaffolding added.
* Solver recovery is seeded with the airframe hover thrust, so the in-solver
  retry can no longer start from a zero-thrust (free-fall) guess.
* A missed wall-clock deadline is reported separately from solver status
  (``SolverDiagnostics.deadline_missed``) and no longer counts toward the
  failsafe threshold: it measures host scheduling, not the optimiser.
* Media assets are deferred until a simulation path exists; the RViz camera
  framing for captures ships in ``rviz/nmpc.rviz``.
* Contributors: Ali-Eimaan.

* Forthcoming: 1.0.0 once all nine acceptance criteria (A1-A9) are green.
