^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uav_mpc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-08-06)
------------------
* Initial release scaffolding.
* Implementation of milestones M1-M10 per the deepseek specification:
  frames and dynamics, symbolic model, solver generation, trajectories,
  solver wrapper, lifecycle node, launch files, CI workflows, analysis and
  media scaffolding, and documentation.
* Decouple the controller from ``px4_msgs`` behind a runtime vehicle backend
  (``generic`` by default, ``px4`` optional), required because ``px4_msgs`` is
  not released for ROS 2 Lyrical Luth.
* Integrate acados 0.6.0 with a CasADi 3.7.2-generated SQP-RTI solver
  (content-hashed codegen output, committed model hash).
* Round 1 review fixes (R1-1 .. R1-15): solve-time budgets from
  ``acceptance_criteria.yaml``, hover test gating, CI lint paths, frame
  conversion edge cases, and more; all verified in
  ``.deepseek/FIX_REPORT.md``.
* Round 2 review fixes (R2-4, R2-5, R2-9, R2-10, R2-12): yamllint targets
  real directories, RViz config is no longer a placeholder, CITATION.cff
  added, requirements pins marked verified, release scaffolding added.
* R2-16 (recovery seeded with airframe hover thrust) was implemented but
  reverted together with the acados 0.6.0 build work pending maintainer
  review of ``.deepseek/ACADOS_BUILD_FINDINGS.md``; it does not depend on
  the solver-instability decision and can be re-applied cleanly.
* R2-8 (media assets) is deferred behind R2-1 (simulation) and R2-5; the
  RViz camera framing for captures now ships in ``rviz/nmpc.rviz``.
* Contributors: Ali-Eimaan.

* Forthcoming: 1.0.0 once all nine acceptance criteria (A1-A9) are green.
