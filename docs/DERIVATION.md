# Derivations — index

**SKELETON.** This file is the readable entry point; the LaTeX sources under
[`derivations/`](derivations/) carry the full detail. Fill both — the `.md` is what a busy
advisor skims, the `.tex` is what they check when they decide you are serious.

> TODO(deepseek): every section below is an outline. Write the actual mathematics. Do not
> paste equations you have not verified; every result here must be reproducible from the
> reference cited beside it.

---

## 1. Notation and frames

- [ ] World frame $\mathcal{W}$: ENU. Body frame $\mathcal{B}$: FLU. State the handedness
      explicitly and give the rotation convention (body-to-world, $R \in SO(3)$).
- [ ] PX4 uses NED/FRD. Give the two fixed rotations and note that they are applied *only* at
      the message boundary in `nmpc_node`.
- [ ] Quaternion convention: Hamilton, $(w, x, y, z)$, body-to-world. State it once, loudly —
      half of all quadrotor bugs are a convention mismatch.

## 2. Rigid-body dynamics on SE(3)

- [ ] Newton–Euler derivation of $\dot p, \dot v, \dot R, \dot\omega$.
- [ ] Quaternion kinematics $\dot q = \tfrac12 q \otimes [0, \omega]$ and the Euler-angle
      alternative with its singularity.
- [ ] Rotor drag term and why it is applied in the body frame.
- [ ] Full detail: [`derivations/quadrotor_se3_dynamics.tex`](derivations/quadrotor_se3_dynamics.tex)

## 3. Control allocation

- [ ] The quad-X mixer, with the motor-numbering diagram matching PX4's convention.
- [ ] Invertibility and the clamped inverse used by `allocateInverse`.

## 4. Differential flatness

- [ ] Flat outputs $\sigma = (x, y, z, \psi)$.
- [ ] The map $\sigma^{(0..4)} \mapsto (x, u)$, including the body-rate expression from the
      jerk projection and the angular-acceleration expression from the snap projection.
- [ ] Where the map degenerates (free-fall, $\|a + g e_z\| \to 0$) and how the code handles it.
- [ ] Full detail: [`derivations/differential_flatness.tex`](derivations/differential_flatness.tex)

## 5. NMPC formulation

- [ ] The OCP as implemented: cost, constraints, horizon, discretisation.
- [ ] Why a nonlinear least-squares quaternion-error residual rather than a linear one.
- [ ] Terminal cost choice and what it does (and does not) guarantee. Be precise here — do not
      claim nominal stability you have not established.
- [ ] RTI: one SQP iteration per sample, the preparation/feedback split, and the resulting
      suboptimality argument.
- [ ] Full detail: [`derivations/nmpc_formulation.tex`](derivations/nmpc_formulation.tex)

## 6. Baseline for comparison

- [ ] The Lee–Leok–McClamroch geometric controller, stated in the same notation, used as the
      reference the NMPC is compared against in the README plots.

## 7. Thesis hook

- [ ] One paragraph: this per-agent NMPC is the building block for the distributed MPC-CBF in
      `transition-viable-swarm`. Name the interface that carries over (the OCP structure and
      the flatness-based reference) and the one that does not (the centralised solve).

---

## References

> TODO(deepseek): full bibliographic entries. At minimum:
> Mellinger & Kumar (ICRA 2011) — minimum snap; Lee, Leok & McClamroch (CDC 2010) — geometric
> tracking on SE(3); Verschueren et al. (2021) — acados; Diehl et al. (2005) — real-time
> iteration; Faessler, Falanga & Scaramuzza (RA-L 2017) — rotor drag and flatness.
