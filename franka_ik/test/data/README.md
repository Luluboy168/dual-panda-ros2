# Stage-0 IK pose corpus

`corpus_v1.json` is the deterministic, solver-independent ground-truth corpus for the
`franka_ik` implementation plan. It contains 3,100 single poses and 20 traces of 200 steps, for
7,100 entries total:

| Category | Entries | Construction |
| --- | ---: | --- |
| `reachable_random` | 2,000 | Limit-respecting joints sampled uniformly with a 0.02 rad inset, then FK. |
| `reachable_near_limit` | 500 | One to three joints within 0.01 rad of a stop, then FK. |
| `singular` | 200 | A rank-deficient family with `q4=-0.467002423653...` and `q5=0`, verified from the PyKDL Jacobian. |
| `unreachable_far` | 200 | A reachable FK pose translated 1.5 m along a fixed-seed random axis and rejected until it is beyond the full kinematic path length. |
| `unreachable_limits` | 200 | A geometrically valid folded pose whose fixed-q7 shoulder-to-wrist distance cannot be produced by any permitted joint-4 value. |
| `drag_traces` | 4,000 | Smooth, fixed-q7 FK poses with at most 2 mm, 1 degree, and 0.15 rad per joint between adjacent steps. |

## Oracle and reproducibility

The generator implements the URDF transform chain directly in pure Python using only the standard
library. It multiplies each fixed joint-origin transform by that joint's local-z rotation and ends
with the fixed 0.107 m `panda_link7` to `panda_link8` transform. It does not use the C++ FK path or
an IK solver.

When `PyKDL` is available, every one of the 7,100 FK evaluations is independently recomputed with
`PyKDL.ChainFkSolverPos_recursive`. Generation fails if any homogeneous-transform element differs
by more than `1e-12`. NumPy is used only with PyKDL to verify that the 200 singular-family
Jacobians have smallest singular value no greater than `1e-12`. The required verification command
on the Jazzy host is:

```bash
python3 franka_ik/test/data/generate_corpus.py --check --require-pykdl
(cd franka_ik/test/data && sha256sum --check corpus_v1.sha256)
```

To regenerate intentionally:

```bash
python3 franka_ik/test/data/generate_corpus.py --require-pykdl
```

The fixed seed is recorded in the JSON. JSON formatting and key order are fixed, and
`corpus_v1.sha256` uses `sha256sum` format. A corpus update must regenerate both files and be
reviewed as a ground-truth change.

## Entry schema

Every entry has:

- `id`: stable category/sequence identifier;
- `category`: one of the six categories above;
- `expected_result`: the solver-independent expected service classification;
- `seed_positions`: seven finite, limit-respecting values in joint1-to-joint7 order;
- `target_pose.position`: `[x, y, z]` in `panda_link0`;
- `target_pose.orientation_xyzw`: a canonical unit quaternion for `panda_link8`.

For reachable entries and trace steps, `seed_positions` is also the exact known FK solution. For
unreachable entries it is only a valid request seed. Extra category-specific fields record the
near-limit joints, far translation, singularity type, or limit-classification geometry. No stored
joint vector lies outside the Panda limits; in particular every stored joint-4 seed is inside
`[-3.0718, -0.0698]`.

## Why `unreachable_limits` is ground truth

The shoulder-to-wrist-centre distance is a sinusoid of joint 4. The generator derives its
unconstrained folded minimum from three independent FK evaluations. That minimum is approximately
0.066 m. Over the permitted joint-4 interval, the minimum is approximately 0.201 m. Each
`unreachable_limits` target is constructed within 0.01 rad of the unconstrained folded minimum,
and generation requires at least 0.10 m separation from the valid-q4 minimum.

The target therefore lies inside the arm's unconstrained geometric sphere but, with the entry's
fixed q7 redundancy value, requires a joint-4 angle above its upper stop (or its equivalent below
the lower stop). The out-of-range construction angle is retained as
`construction_joint_positions` for audit but is never used as a request seed.

The singular poses are mathematically reachable because each has a known valid FK solution. A
specific numeric-backend non-success code, if any, is a Stage-2 measured solver outcome rather
than part of this Stage-0 ground truth.
