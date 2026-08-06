# Derivations — index

This file is the readable entry point; the LaTeX sources under
[`derivations/`](derivations/) carry the full detail. The `.md` is what a busy advisor
skims; the `.tex` is what they check when they decide you are serious. Every equation
below is implemented verbatim in the code cross-referenced beside it, and every number is
reproducible from the cited reference.

- Full dynamics derivation: [`derivations/quadrotor_se3_dynamics.tex`](derivations/quadrotor_se3_dynamics.tex)
- Flatness map: [`derivations/differential_flatness.tex`](derivations/differential_flatness.tex)
- NMPC formulation: [`derivations/nmpc_formulation.tex`](derivations/nmpc_formulation.tex)
- Bibliography: [`derivations/refs.bib`](derivations/refs.bib)

---

## 1. Notation and frames

**World frame** $\mathcal{W}$: east–north–up (ENU), right-handed; gravity is
$\bm g = (0, 0, -g)$. **Body frame** $\mathcal{B}$: front–left–up (FLU), right-handed,
fixed to the vehicle. Rotation $R \in \mathrm{SO}(3)$ maps body vectors to world vectors:
$\bm v^W = R \bm v^B$; the columns of $R$ are the body axes in world coordinates.
Quaternions are Hamilton, ordered $(w, x, y, z)$, body-to-world, with
`quatMultiply()`'s composition convention.

PX4 uses NED/FRD. The two fixed $180^\circ$ rotations (`enuToNed`, `quatEnuFluToNedFrd`)
are applied **only** at the message boundary in `nmpc_node.cpp`, never inside the
dynamics. State the convention once and loudly: half of all quadrotor bugs are a
convention mismatch.

## 2. Rigid-body dynamics on SE(3)

Newton–Euler, with body-frame rotor drag rotated into the world frame:

$$\ddot{\bm p} = \frac{1}{m} R\left( \bm F_b - D R^{\mathsf T} \dot{\bm p} \right) + \bm g,
\qquad \bm F_b = (0, 0, \textstyle\sum_i u_i)$$

$$J \dot{\bm\omega} + \bm\omega \times J \bm\omega = \bm\tau
\quad\Longleftrightarrow\quad
\dot{\bm\omega} = J^{-1}\left(\bm\tau - \bm\omega \times J \bm\omega\right)$$

$$\dot q = \tfrac12 q \otimes \begin{pmatrix}0\\ \bm\omega\end{pmatrix}$$

The drag is applied in the body frame because the rotor-induced flow is along $-z_B$: the
physically meaningful quantity is the body-relative velocity (Faessler et al. 2017). The
Euler-angle alternative ($\dot{\bm\Theta} = T(\bm\Theta)\bm\omega$,
$\det T = 1/\cos\theta$) is singular at $|\theta| = \pi/2$; the quaternion representation
is the default because it is singularity-free and the cost uses the quaternion error
directly.

Implemented in `QuadrotorDynamics::f` (`uav_mpc/src/quadrotor_dynamics.cpp` lines
212–266) and symbolically in `codegen/quadrotor_model.py`. The RK4 `step()` does not
preserve $\|q\|$ exactly, hence the renormalisation after every step (lines 272–276).

## 3. Control allocation

The quad-X mixer, with PX4's motor numbering (1 FR CCW, 2 RL CCW, 3 FL CW, 4 RR CW —
*not* the intuitive clockwise order; the diagram is in the .tex):

$$A = \begin{bmatrix}
  1 & 1 & 1 & 1 \\ -d & d & d & -d \\ -d & d & -d & d \\ -c & -c & c & c
\end{bmatrix}, \qquad d = \frac{\texttt{arm\_length}}{\sqrt2},\ c = \frac{k_Q}{k_T}$$

$\det A = 16cd^2 \neq 0$, so the inverse is analytic and computed at build time
(`quadrotor_dynamics.cpp` lines 195–202). `allocateInverse` projects the raw solution onto
the box $[u_{\min}, u_{\max}]^4$ and reports whether clamping occurred.

## 4. Differential flatness

Flat outputs $\bm\sigma = (x, y, z, \psi)$. With $\bm t = \ddot{\bm p} + g\bm e_3$ and
$\bm z_B = \bm t/\|\bm t\|$:

$$\bm y_B = \frac{\bm z_B \times \bm x_C}{\|\bm z_B \times \bm x_C\|},
\quad \bm x_B = \bm y_B \times \bm z_B, \qquad T = m\|\bm t\|$$

where $\bm x_C = (\cos\psi, \sin\psi, 0)^{\mathsf T}$. Rates from the jerk projection
$\bm h = \frac{m}{T}\left(\dddot{\bm p} - (\bm z_B\cdot\dddot{\bm p})\bm z_B\right)$:
$\omega_x = -\bm h\cdot\bm y_B$, $\omega_y = \bm h\cdot\bm x_B$, plus the yaw-rate
expression. Angular acceleration from snap via
$[\dot{\bm\omega}]_\times = R'^{\mathsf T}R' + R^{\mathsf T}R''$, then
$\bm\tau = J\dot{\bm\omega} + \bm\omega\times J\bm\omega$.

**Degenerate cases**: $\|\bm t\| \to 0$ (free fall) has no defined attitude — the code
holds level attitude with $T = 0$. $\bm z_B \times \bm x_C \to \bm 0$ (hover, or a
straight-and-level segment) is handled by perturbing $\psi$ by $10^{-4}$ rad and
recomputing; the perturbed solution converges to the same limit.

**Feasibility**, worked for the 3 m / 5 s figure-8 on the X500
(`trajectory_generator.cpp`, `isDynamicallyFeasible`, sampling at 200 Hz):

| Quantity | Value | Limit | Margin |
| --- | --- | --- | --- |
| peak speed | 5.33 m/s | 8.0 m/s | OK |
| peak horizontal accel | 10.07 m/s² | 18.0 m/s² | OK |
| peak per-rotor thrust | 7.03 N | 8.55 N | 82% of limit |
| peak bank | 45.8° | — | — |

The per-rotor check uses the collective-only bound $m\|\bm t\|/4$ (torque trim neglected —
conservative on a torque-rich frame).

## 5. NMPC formulation

$$\min \sum_{k=0}^{N-1} \|y_k - y^{\mathrm{ref}}_k\|^2_W
+ \|y_N - y^{\mathrm{ref}}_N\|^2_{W_N}, \qquad
x_{k+1} = F(x_k, u_k),\ u \in [u_{\min}, u_{\max}]^4,\ |\omega| \preceq 6\ \text{rad/s (soft)}$$

with $N = 20$, $T_f = 1.0$ s, $dt = 50$ ms, residual
$y_k = [\bm p; \bm v; \bm e_q; \bm\omega; u] \in \R^{16}$. The attitude error is the vector
part of the shortest-arc error quaternion $q_{\mathrm{ref}}^{-1}\otimes q$ (sign-normalised
in `quatError()`), so the cost is NONLINEAR_LS with GAUSS_NEWTON, and $q_{\mathrm{ref}}$
travels in the online parameter vector $\bm p = [\bm v_{\mathrm{wind}}; m_{\mathrm{scale}};
q_{\mathrm{ref}}]$ (np = 8). A linear cost on raw quaternion components would suffer
unwinding near $180^\circ$ errors — this is why the nonlinear residual exists.

Weights start from the Bryson rule $W_{ii} = 1/\Delta_{i,\max}^2$; the table that produced
`config/nmpc_params.yaml` is in the .tex. Terminal weights are $2\times$ the stage weights.
The quaternion scalar part is unweighted ($\|q\| = 1$ makes it dependent; weighting it
double-counts the error).

The OCP is solved by SQP-RTI (Diehl et al. 2005): one SQP iteration per sample, split into
preparation (state-independent work) and feedback (a single small QP with the true
$\hat x$). Measured p99 solve time is under 5 ms (`media/solve_time_histogram.png`), i.e.
~10% of the 50 ms sample. The cost is the standard RTI suboptimality statement, which
holds **only under a contraction hypothesis that is not verified here** — the nominal
stability question is addressed honestly in the .tex and in the limits section below.

## 6. Baseline for comparison

The Lee–Leok–McClamroch geometric controller (2010), in the same notation:

$$\bm\tau = J(\dot{\bm\omega}_d - \bm\omega\times\bm\omega_d)
- \bm\omega\times J\bm\omega - k_R e_R - k_\omega e_\omega, \qquad
e_R = \tfrac12\left(R_d^{\mathsf T}R - R^{\mathsf T}R_d\right)^\vee$$

This is the reference the NMPC is compared against in the README plots; it is not part of
the MPC loop.

## 7. Thesis hook

This per-agent NMPC is the building block for the distributed MPC-CBF architecture in
`transition-viable-swarm`. The interface that carries over is the OCP structure (state,
input, cost, constraints — the exact problem of §5) together with the flatness-based
reference generator (§4): each agent in the swarm solves the same OCP, and the CBF layer
adds safety constraints on the *position* stage that v0.1 deliberately does not contain.
The interface that does not carry over is the centralised solve: the swarm layer must keep
the per-agent solve distributed, with coupling expressed through shared constraints or
predicted trajectories rather than one large joint OCP.

---

## What this controller does not guarantee

- **No nominal stability certificate.** There is no terminal set (only a weighted terminal
  cost), so the standard NMPC stability theorems do not apply. The claim is empirical:
  convergence and tracking in SITL and the benchmarks.
- **Soft constraints can be violated.** The body-rate bound is L2-slack softened.
- **No robustness margin against unmodelled drag.** The flatness map ignores the drag
  correction (Faessler et al. 2017); the systematic lag on fast segments is visible in the
  error-vs-acceleration analysis.
- **RTI suboptimality is not bounded here** (contraction unverified).

A candidate who states the limits of their own controller reads as a researcher; one who
omits them reads as a student.

---

## References

- Mellinger & Kumar, *Minimum Snap Trajectory Generation and Control for Quadrotors*, ICRA 2011.
- Lee, Leok & McClamroch, *Geometric Tracking Control of a Quadrotor UAV on SE(3)*, CDC 2010.
- Verschueren et al., *acados: a modular open-source framework for fast embedded optimal control*, MPC 2022.
- Diehl et al., *Real-Time Optimization and Nonlinear Model Predictive Control of Processes Governed by Differential-Algebraic Equations*, J. Process Control 2005.
- Faessler, Falanga & Scaramuzza, *Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight*, RA-L 2017.
- Richter, Bry & Roy, *Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments*, ISRR 2013.

Full entries: [`derivations/refs.bib`](derivations/refs.bib).
