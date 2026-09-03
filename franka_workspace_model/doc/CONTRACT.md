# The workspace model, as a contract

This is the operator-facing and integrator-facing contract for
`franka_workspace_model`. Every path named in an error message this package can
emit points into this document or into a file beside it. Nothing here refers to
anything outside the installed package.

Read section 1 if a message sent you here. Read section 2 before editing
`cell_model_v1.yaml`. Read section 6 before calling the API.

---

## 1. The rules that stop you

### The URDF-disagreement rule

The cell file records two base poses per arm: `urdf_base_pose`, copied verbatim
from the robot description, and `measured_base_pose`, what the tape says. If
they disagree by more than the recorded tolerance, **the model refuses to load**.

That is deliberate, and the fix is never to edit the tolerance until the message
goes away.

Forward kinematics — the thing that decides where the checker believes the links
are, and the thing the real controllers and `robot_state_publisher` use — comes
from the description. A cell file that "knows better" than the description
produces a checker that is confidently wrong about link positions: it computes
clearances for a robot that is not the one moving.

**On disagreement: stop, and report to whoever owns the cell.** Present the
measured value, the description's value, and the difference. There are exactly
three legitimate ways forward:

1. the description is right and the measurement is re-taken;
2. the description is wrong, and it is corrected in its own reviewed change,
   with this package's derived geometry regenerated in the same commit;
3. the disagreement is accepted, quantified and documented — the tolerance
   fields are widened *deliberately and visibly*, and `measurement_status` is
   set to say what is actually known.

There is no override flag, and there will not be one.

### The switchable end effector

Every operator launch runs with the grippers disabled, so the robot description
carries **no end-effector geometry at all**. The cell file's `end_effector`
block is the only thing standing between a mounted tool and a checker that
believes the flange is bare.

The block separates the *declaration* from the *switch*:

- `profile` names the tool (`none` or `robotiq_2f85`) and carries its geometry
  whether or not the tool is fitted;
- `present` says whether the tool is physically mounted and therefore checked.

So fitting the gripper is a one-word edit plus a `revision` bump — not a
geometry-authoring exercise performed under time pressure with the hardware
already bolted on.

**An enabled end effector whose geometry is still `to_be_derived` is a load
error, never a warning.** The failure mode of an undersized end-effector volume
is not "the model is a bit optimistic": the checker would model the mounted tool
as a zero-size capsule and report the most collision-prone part of the arm as
clear.

Before setting `present: true`, derive the capsule and record it:

| # | Obligation |
| --- | --- |
| 1 | **Cite the source.** `provenance` names the datasheet by document revision, page, and the figure or table the dimensions came from. "Robotiq datasheet" alone is not provenance. |
| 2 | **Bound the whole motion envelope**, not the shipping pose. The fingers move; the capsule must contain the union over the entire stroke. |
| 3 | **Include the coupling plate** between `link8` and the gripper body, and any cabling standing proud of it. |
| 4 | **Get the `link8` frame right**: origin on the flange face, `+z` outward from the arm. A 90-degree error about `+z` is invisible in the numbers and wrong in the model. |
| 5 | **Record `containment: conservative` and a real `containment_margin`.** "I rounded up to be safe" is not a proof. |
| 6 | **Do not double-count the 30 mm.** Description radii already carry a 30 mm inflation; an end-effector capsule is not a description volume and carries none. See section 4. |

Until all six are done, leave `present: false` and do not mount the tool.

### The allowed volume

`allowed_volume` is the box the arms must stay **inside**. Under the table-top
convention its floor `z_min` is the table top *and* the mounting plane: the
table is not a declared obstacle, it is the floor of the permitted region.

`z_max` is the ceiling. "No practical ceiling" is expressed as a value beyond
reach rather than by omitting the key, so the schema stays exact-keyed. An arm's
collision volumes cannot rise above 1.401 m over its own base — the sum of the
chain's translations, 0.333 + 0.316 + 0.0825 + 0.384 + 0.088 + 0.107 = 1.3105,
plus `link8`'s extent and radius — so a `z_max` above that provably never binds,
and the loader says so in its diagnostics when a declared ceiling *can* bind.

The checker always evaluates **all six faces**. A checker that skipped the
ceiling because this cell's ceiling never binds would silently stop enforcing
any ceiling a future cell declares.

### Margins

A margin is **extra clearance on top of the 30 mm already inflated into every
collision radius by the robot description**. Write that sentence wherever a
margin number appears. The physical surface-to-surface clearance at the
violation threshold is:

```
margin + 0.06   robot against robot          (both volumes inflated)
margin + 0.03   robot against a declared solid, a keep-out zone,
                or the allowed-volume boundary   (one volume inflated)
margin + 0.00   any pair involving an end-effector volume, which is not a
                description volume and carries no built-in inflation
```

The failure this warning exists to prevent is somebody adding 30 mm a second
time "to be safe", which turns a 0.05 m cross-arm margin into an effective
0.11 m fence and makes the cell unusable — or reading 0.02 m as 20 mm of real
clearance and believing the fence is tighter than it is.

The margins shipped in `cell_model_v1.yaml` are **placeholders**: the
measurement tolerances they should be derived from have not been answered. The
file says so in its own `rationale` field, which is a required field for exactly
this reason. Replacing them is a value edit plus a `revision` bump.

---

## 2. `cell_model_v1.yaml`, key by key

Fourteen top-level keys, none optional, no unknown keys accepted. The file is
read once, hashed, and never re-read or patched.

| Key | What it is |
| --- | --- |
| `schema_version` | Exactly `1`. |
| `model_id` | `^[a-z][a-z0-9_]{2,63}$`; echoed in every result. |
| `revision` | `>= 1`; bump it on every content change. |
| `measured_on` | `YYYY-MM-DD`, and a real calendar date. |
| `measured_by` | 1–64 printable ASCII characters. |
| `units` | Exactly `{length: m, angle: rad}`. The file declares its units rather than implying them. |
| `sources` | Four path/hash pairs — see below. |
| `cell_frame` | `{name: cell, anchors: {dual, single}}`. |
| `arms` | One or two entries; see below. |
| `allowed_volume` | The containment box; eleven keys. |
| `environment` | Declared keep-away solids. `[]` is normal. |
| `keep_out` | Policy zones. Each carries a written `reason`. |
| `margins` | Seven keys; see section 1. |
| `policy` | Nine keys; see section 3. |

### `sources`

```yaml
sources:
  urdf_xacro:                franka_description/robots/real/dual_panda_arm.urdf.xacro
  urdf_xacro_sha256:         "<64 lowercase hex>"
  srdf_xacro:                franka_moveit_config/srdf/dual_panda.srdf.xacro
  srdf_xacro_sha256:         "<64 lowercase hex>"
  joint_limit_policy:        franka_example_controllers/config/panda_joint_limits_v1.yaml
  joint_limit_policy_sha256: "<64 lowercase hex>"
  link_geometry:             link_geometry_v1.yaml
  link_geometry_sha256:      "<64 lowercase hex>"
```

**Every hash must match the file on disk, or the model refuses to load.** A
mismatch means the model was derived from a different robot than the one you are
about to check.

`link_geometry` resolves next to the cell file. The other three are repository
relative: the loader resolves the cell file's real path, then walks upward until
it finds a directory containing all three. A model therefore loads from a source
checkout, or from an install tree whose files are symlinks into one (which is
what `colcon build --symlink-install` produces). If no such directory exists the
load fails with a message saying so, rather than checking against geometry it
cannot verify.

### `arms[]`

Five keys: `arm_id`, `base_link` (which must be `<arm_id>_link0`),
`urdf_base_pose`, `measured_base_pose`, `end_effector`.

`measured_base_pose` carries `tolerance_m`, `tolerance_rad` and a
`measurement_status` of `measured` | `assumed` | `inherited_from_urdf`. An
unanswered question yields `assumed`, never a silent default value.

### Environment solids and keep-out zones

Both use the same five geometry kinds, and the key set of an entry is its common
keys plus exactly the keys its `kind` declares:

| `kind` | Geometry keys |
| --- | --- |
| `box` | `pose`, `size` |
| `cylinder` | `pose`, `length`, `radius` |
| `sphere` | `pose`, `radius` |
| `capsule` | `a`, `b`, `radius` |
| `plane_halfspace` | `normal`, `offset` — the forbidden region is `n·p < offset` |

A `cylinder` is promoted at load to its minimal enclosing capsule: same axis,
same radius, hemispherical caps. The promotion is conservative in the safe
direction and is reported in the load diagnostics, so the model is never
silently larger than the solid you declared.

`environment` entries and `keep_out` zones are *obstacles*: the arm must stay
**outside** them. `allowed_volume` is the opposite. The two are not variants of
one thing, which is why they carry different `Contact.kind` values.

---

## 3. Cross-field rules

These are the rules that catch a file which is individually well-formed and
collectively wrong.

| Rule | What it requires |
| --- | --- |
| Arm count | `dual` requires exactly two arms, `single` exactly one; `arm_id` values are unique. |
| Unique ids | Every id across `allowed_volume` + `environment` + `keep_out` is unique — the box's id is the prefix of every containment contact's `b` field, so a collision there is a wire-format collision. |
| `applies_to` | Names declared arms, or is the single string `all`. |
| Pair deltas | Name links that exist in the derived geometry, never the same link twice, and never the same pair in both lists. |
| Base pose | See "The URDF-disagreement rule". |
| Hashes | See `sources`. |
| Inflation | `margins.urdf_builtin_inflation` must equal the derived geometry's recovered `safety_distance`. It is documentation; it is never applied. |
| Dual anchor | Exactly the identity, which is what makes the cell frame coincide with `base_link`; and in a dual model the two `urdf_base_pose` values are antisymmetric about the origin. |
| Single anchor | Equals the negation of the declared arm's `urdf_base_pose`, so the single profile is arm-agnostic: a `panda2`-only session is checked in the cell frame, not in `panda1`'s. |
| Frames | Every solid's `frame` equals `cell_frame.name`. |
| End effector | `volumes` is non-empty exactly when `profile != none`; `present: true` requires a named profile and fully derived volumes. |
| Box bounds | `x_min < x_max`, `y_min < y_max`, `z_min < z_max`. An inverted box would make every configuration a violation, and a model that refuses everything trains the operator to switch it off. |
| Base placement | Every arm's measured base lies inside the box laterally with at least `margins.environment` of slack, and its `z` equals `z_min`. A cell describing arms mounted outside their own permitted region is self-contradictory before any pose is checked. |
| Pinned policy | `default_mode: swept`, `fail_closed: true`, `environment.enabled: true`, `containment.enabled: true`, and `cross_arm.enabled` true for two arms and false for one. |

Every one of these raises with a message naming the key, what was found, and
what was expected. Two different defects never produce the same message.

---

## 4. What the checker checks

A *sample* is one complete joint configuration for every arm. Five steps, in
this order, with a fixed evaluation order inside each so that ties break
identically every run:

| # | Step | Geometry | This cell |
| --- | --- | --- | --- |
| 1 | **Joint limits**, against the referenced policy file | — | 14 joints |
| 2a | **Self-collision**, the SRDF matrix plus the recorded deltas | **mesh bodies** | 16 link pairs per arm → **26 body pairs per arm**, 52 for the cell |
| 2b | **Structure**: the pedestal against each arm | **mesh bodies** vs the declared box | 18 volume pairs → the bodies behind them |
| 3 | **Cross-arm**: every body of one arm against every body of the other | **mesh bodies** | 81 link pairs → **121 body pairs** |
| 4a | **Containment**: every non-exempt body against every face of the box | **mesh bodies** | 18 volumes → **20 bodies** × their unmasked faces |
| 4b | **Environment**: every moving volume against every declared solid | capsules | `environment` is `[]`: inert |
| 5 | **Keep-out**: every moving volume against every *enabled* zone | capsules | the one zone is disabled: inert |

Contacts name **mesh bodies**, never links: `panda1_link5_collision_2_st` and
`panda1_link8_flange`, where the capsule model said `panda1_link5_v1`. Same
field, same type, same JSON, new values in an existing namespace. `base_link_v0`
keeps its volume id because the pedestal is a **declared box** in the cell file,
not derived geometry.

`link5` expands to three bodies and not two, because `mj_dual.xml` ships it
decomposed into three collision pieces and this model keeps it that way rather
than re-convexifying it — the hull of the three is 3250 cm³ against 2076 cm³
summed, and that 36 % of phantom volume sits exactly at the wrist.

**Steps 4b and 5 were NOT converted, and the silence about that would have been
the defect.** Both are inert in this cell — `environment` is `[]` and the
midplane zone is `enabled: false` — so neither evaluates any geometry today, and
converting a step that runs on nothing is a change nobody could check. The cost
when one of them is populated is: `environment` needs the mesh-body-against-
declared-solid distance, which for a box is the same GJK call the pedestal makes
and for a sphere or a cylinder is a support call; `keep_out` needs the
mesh-body-against-zone test. Until then they are inert, and this paragraph is
here so that a reader does not have to infer it from an omission.

### Clearance is real air

The number a contact reports is

```
clearance = gjk(body_a, body_b)
```

and **nothing is subtracted from it.** No undercut term, because the bodies
contain the visual shell by construction. No coverage term. No capsule radius —
`r_a` and `r_b` belong to the broad phase and never appear in a reported number.

What that costs, and what it buys, on the pair everything turns on. At the ready
pose `link5` against `link7` measures:

| model | value | verdict at 20 mm |
| --- | ---: | --- |
| the raw collision meshes (they undercut the shell) | 23.2523 mm | allowed |
| the collision meshes minus the per-link undercut | 16.1433 mm | **REFUSED** |
| **the shipped bodies** | **21.7786 mm** | allowed by 1.78 mm |
| true shell-against-shell | 21.7955 mm | — |

0.017 mm of pessimism, against 7.109 mm for the honest scalar-subtraction
alternative — which refuses the arm's own home pose.

**Penetration reports `0.0` and no depth.** GJK does not compute one and the
model does not need one: penetration is always a rejection, the jog fence clamps
from the safe side where GJK is exact, and the IK filter discards. A consumer
that *ranks* candidates by clearance sees every penetrating candidate tie. The
capsule fence reported a signed overlap here; that number was not a distance any
surface had.

**A GJK iteration-cap failure is a refusal.** The cap is reachable, and only on
pairs within about a nanometre of contact, where refusing is the correct verdict
anyway. It surfaces through the existing fail-closed `GeometryError` path.

### What the fence still does not know

Three things, and they are stated here rather than in a footnote.

**`link8` is an envelope, not metal.** `franka_description` ships no `link8`
mesh, so its body is a hull circumscribing the URDF's own primitives at metal
radius. It provably overlaps `link6` and `link7`, and it is the binding pair on
more uniform draws than every other pair combined. That cost is the envelope's,
not the metal's. One caliper reading of the flange boss retires it.

**The whole fidelity chain terminates in art.** The visual `.dae` shells are
Blender files from 2018; the collision meshes are Blender-2.79 STL exports.
Nothing here is a Franka-certified envelope and `franka_description` ships none.
The claim this package makes is *"every body contains this description's own
collision geometry and its own visual shell, exactly, and here is the
certificate"* — never *"this is the metal"*. That is strictly more than the
capsule set could say.

**`link5` against `link7` is held apart by a joint limit with 1.86° of room.**
First metal contact is at `j6 = −0.049945` rad, below the URDF lower limit of
−0.0175. A mis-zeroed `j6`, a limit-enforcement error, or a description edited
to FR3 ranges puts real metal into real metal. That is a robot risk rather than
a modelling one, and it is recorded here because the measurement found it.
`mj_dual.xml`'s joint ranges ARE the FR3's on a Panda chain — this package never
reads them, and neither should anything else.

### The acceptance census

`test/test_acceptance_census.py` draws twenty thousand uniform configurations at
a pinned seed and asks, of every one the fence ACCEPTS, whether the metal is
really there — against an oracle that shares no kinematics and no distance code
with the checker.

It exists because the branch that made this fence looser passed every other test
in this package. Every test here asked "does the checker refuse the poses I
wrote down"; none asked "what does it accept". Four hard criteria say nothing
unsafe is accepted, three anti-vacuity criteria say the fence is still a fence,
and it covers **all 36 link pairs** rather than the 16 the SRDF leaves enabled —
which is how the `link2`/`link6` hole was found. It has no marker, no `skipif`
and no environment opt-out, and its seven constants are asserted exactly by a
second test, so weakening it means editing two places and explaining both.

**A pair violates when `clearance < margin`.** The comparison is strict: a pair
exactly on its margin passes. `clearance` is a surface-to-surface distance for
the keep-away steps, negative on penetration, and a distance to the nearest box
boundary measured from inside for the containment step, negative on protrusion.
Both are the same kind of number — positive means slack — which is why one
comparison rule and one `min_clearance` cover both.

`CheckResult.min_clearance` is **margin-adjusted**: the minimum over everything
evaluated of `clearance - required`. A pair 10 mm apart under a 50 mm margin
contributes `-0.040`, not `+0.010`. Joint-limit contacts contribute radians past
the limit, and only when they violate; a consumer that needs a metric clearance
filters `contacts` by `kind`.

**On a PASSING configuration it is a lower-bounded estimate, not the exact
minimum.** The broad phase culls any pair whose certified bound already exceeds
its own margin, and such a pair can hold the true tightest slack. Two properties
hold instead:

- it is **exact whenever any pair is inside its margin** — that pair is never
  culled and it carries the minimum, so every result with a contact in it is
  exact;
- every culled pair contributes its **certified lower bound**, which is valid,
  already computed and free, so the reported value is a true lower bound on the
  tightest slack rather than an unbounded over-estimate.

One consequence a consumer should know about: widening a margin can make the
reported minimum *rise*, because a pair that flips from culled to evaluated
stops contributing its conservative bound and starts contributing its exact
distance. The verdict is monotone; the reported number is monotone only where it
is exact. A consumer that ranks candidates — the IK filter, the ghost, the web
scene — sees a slightly different number than the capsule fence gave, always on
the safe side.

### A ruled margin on one pair

`policy.self_collision.pair_margins` is a list of
`{a, b, margin, reason}`. It rules a self-collision margin for ONE link pair,
and every body pair of that link pair inherits it — the same expansion rule
`extra_enabled_pairs` uses.

**It exists because a margin can be the wrong instrument for a pair.** A
margin's job is to absorb model error and calibration error. On a pair whose
entire separation range is a couple of centimetres, a twenty-millimetre margin
does not absorb error; it declares most of the manufacturer's own designed
range out of bounds. The alternative — quietly lowering the global margin for
every pair — is how a fence gets weaker without anybody deciding that it
should. This key makes the ruling a diff with a reason attached.

**It is a margin and not an exemption.** The pair is still evaluated, still
reported, still refused inside the ruled distance, and still pinned by the
acceptance census. `reason` is mandatory and non-empty for exactly that
reason, and the load prints the ruled value beside the one every other pair
gets, so an operator reading the console sees the ruling rather than inferring
it from a number that is quietly different.

Load rules, each with its own message:

| Rule | Why |
| --- | --- |
| Names two links of one arm | A self margin is an intra-arm quantity; the cross-arm margin is `margins.cross_arm`. |
| Names a pair the fence actually evaluates | A ruling with no effect is worse than none: it reads as protection. |
| `margin` finite and non-negative | A negative margin is not a margin. |
| `reason` non-empty | An unexplained ruling is indistinguishable from somebody making a red test go away. |
| At most one entry per pair | A pair has one ruled margin or none; two is a question nobody answered. |

The key ships **empty** unless a ruling has been made, and a ruling that
loosens a pair arrives together with the geometry that justifies it — never
before it.

### The broad phase culls against each pair's own margin

The mesh fence bounds every body pair before measuring it:

```
lower_bound(a, b) = seg_seg(A, B) − r_a − r_b   ≤   gjk(body_a, body_b)
evaluate  iff  lower_bound(a, b) ≤ that pair's own margin
```

`r_a` and `r_b` are **bounding-capsule radii**, each the exact maximum
vertex-to-segment distance of its own body. They exist so that a
segment–segment distance can *bound* a body–body distance. **They never appear
in a reported number.** The reported clearance is `gjk(body_a, body_b)`, full
stop; writing `gjk(...) − r_a − r_b` would subtract them a second time, which
on `link5`/`link7` at the ready pose is worth −107 mm and would refuse the home
pose.

The cull is against each pair's **own** margin and never against a global
minimum. Those are different questions the moment margins differ per class: a
self pair at 30 mm can be the global minimum and pass, while a cross-arm pair
at 40 mm violates its own 50 mm margin and is never evaluated. Because the gate
IS the margin, tightening one cannot make the cull unsound and loosening one
cannot make it miss a contact.

**What this does to `min_clearance` on a PASSING configuration.** A culled pair
can hold the true tightest slack, so the reported minimum is a *lower-bounded
estimate* rather than the exact minimum over all pairs. Two properties are
guaranteed instead:

- it is **exact whenever any pair is within its margin** — such a pair is never
  culled, and it carries the minimum;
- every culled pair contributes its **certified lower bound** to the minimum,
  which is valid, already computed, costs nothing, and keeps the reported value
  a true lower bound on the tightest slack rather than an unbounded
  over-estimate.

A consumer that ranks candidates by `min_clearance` — the IK filter, the ghost,
the web scene — therefore sees a slightly different number than the capsule
fence gives, always on the safe side.

### The SRDF pair that is not "Never"

The allowed-collision matrix is read from the SRDF and never restated. One of
its entries is measured false, and the correction lands here rather than there.

`franka_moveit_config/srdf/panda_arm.xacro` and `dual_panda_arm.xacro` both
carry `<disable_collisions link1="${arm_id}_link2" link2="${arm_id}_link6"
reason="Never"/>`. At

```
q = [2.313281, -0.882717, 0.476635, -3.067587, -2.642360, 3.401809, 0.880777]
```

every joint is strictly inside its own URDF limit — the smallest slack is
0.004213 rad on `j4` — and the `link2` and `link6` collision meshes
**interpenetrate by 0.954 mm**, measured with a linear program over the two
hulls' half-spaces that shares no code with this package. On the model's own
volumes the overlap is 1.036 mm. The distance is an intra-arm quantity, so it
does not depend on where the arm is bolted and the same figure holds on either
arm.

Without a delta the model has **no opinion at all** about that pair: it is
disabled in the inherited matrix and is structurally absent from
`_intra_pairs`. At the witness pose the shipped fence does refuse — on
`link2`/`link7` padded capsules, 30 mm of inflation on an unrelated pair — which
is cover, not a check.

`policy.self_collision.extra_enabled_pairs` therefore names
`{arm}_link2`/`{arm}_link6` on both arms, with the measurement written into the
mandatory `reason`. **The SRDF itself is not edited.** It is an inherited file;
a local correction there would be invisible to every other consumer of it, and
the supported route for a cell-specific tightening is the cell file. Enabling a
pair can only ever make the fence stricter, so the delta needs nothing behind
it: a pair that was not evaluated cannot become looser by being evaluated.

`test/test_falsified_srdf_pair.py` pins all of it, including the sentence that
the SRDF still says `reason="Never"`.

### Swept, always

`check_path` and `check_jog` resample at `policy.max_joint_step_rad` and check
every sample including both endpoints, with every margin increased by
`margins.swept_path_extra`. A single 2-degree jog is the case nobody thinks to
doubt and the case where both ends can be clear while the interpolant passes
through contact. `policy.default_mode` exists so the schema is stable, and any
value other than `swept` is refused at load.

### Volumes that are exempt from the floor, and why

Three classes of volume sit permanently below the table top, by construction and
at every joint configuration:

| Volume | Below the table top by | Why |
| --- | --- | --- |
| `base_link_v0` | 0.0866 m (bounding sphere; the cube itself 0.0500 m) | The 0.1 m pedestal cube is centred on the origin, so half of it is below the mounting plane. |
| `panda{1,2}_link0_v0` | 0.0300 m | Capsule at link-frame `z = 0.06` with radius 0.09, and `link0`'s frame *is* the mounting plane. |
| `panda{1,2}_link1_v0` | 0.0900 m | `joint1`'s origin is `(0, 0, 0.333)` and the capsule ends at link-frame `z = -0.333`, so its lower cap sits exactly on the table top and the radius carries it 90 mm through. |

None of these is a defect: they are the 30 mm inflation applied to links whose
collision volumes legitimately begin at the plane the arm is bolted to. The
physical castings do not extend into the table; their inflated capsules do.

A constant cannot be a fence, because no motion can change it. Two exemption
classes, both **computed from the joint chain at load** rather than from any
hand-written list of names:

1. **Fully static** — every joint between the cell frame and the volume's link
   is fixed. Excluded from all per-query steps and checked once at load.
2. **`z`-static** — every joint in the chain is fixed or revolute about an axis
   parallel to cell `+z`. Rotation about a vertical axis cannot change any
   point's height, so these are exempt from the `z_min` and `z_max` faces only,
   and are checked normally against the four lateral faces.

The loader reports every exemption with its constant clearance, and for
`base_link_v0` with both figures in the table above — the conservative bounding
sphere the check is measured with, and the declared cube itself — so that nobody
has to reconcile the two later. Tilt `joint1`'s
axis in the derived geometry and `link1` leaves the second set on its own; that
is what makes the rule derived rather than merely described as derived.

### Fail-closed

Any of the following is a refusal, never an approval and never a partial
verdict: the model fails to load or a hash does not match; a joint name is
unknown, a `q` vector is the wrong length, or any value is non-finite; the
running description does not hash to the recorded digest; the number of arms in
the request does not match the model; any internal numeric error. All of them
raise `WorkspaceModelError`.

---

## 5. `link_geometry_v1.yaml`

Derived, committed, and regenerated in CI. Nothing in it is measured, chosen or
tunable: every number is already in the robot description, and the file's whole
job is to make a description change visible as a diff instead of as a silent
change of behaviour in the checker.

Regenerate it with:

```sh
python3 -m franka_workspace_model.generate_link_geometry \
  --repository-root <the repository root> \
  --urdf-xacro franka_description/robots/real/dual_panda_arm.urdf.xacro \
  --arg arm_id_1:=panda1 --arg arm_id_2:=panda2 \
  --arg hand_1:=false --arg hand_2:=false \
  --arg robot_ip_1:= --arg robot_ip_2:= \
  --arg use_fake_hardware:=true --arg fake_sensor_commands:=false \
  --output <package>/cell/link_geometry_v1.yaml
```

**That argument set is part of the contract**, and the file records it under
`source.xacro_args`. `source.urdf_sha256` is the digest of the *generated URDF
text*, not of the `.xacro` source, so a differently-parameterised expansion is a
different digest even when the geometry is identical. A consumer that wants to
reproduce the digest — or to compare it against a description it generated
itself — must expand with exactly these arguments. `CellModel.xacro_args()`
returns them at runtime.

**The text is canonicalised before it is hashed.** `xacro` opens its output with
a banner naming the absolute path it expanded, so the digest of its raw standard
output would be a property of one machine's directory layout rather than of the
robot. The digest is therefore taken over
`xml.etree.ElementTree.canonicalize(rendered, with_comments=False)` — XML
canonicalisation, C14N 2.0, comments discarded — which removes the banner and
every other serialisation accident. Reproducing the digest is two lines:

```python
from franka_workspace_model.model import urdf_digest

urdf_digest(subprocess.run(['xacro', path, *args], ...).stdout)  # == model.urdf_sha256()
```

The session interlock in `ros/description_interlock` applies the identical
normalisation to the running `robot_description`, so the recorded digest — taken
from a source checkout — and the running one — expanded from the install space —
compare like for like. Without that, fail-closed would mean jogging stayed
disabled on a correct robot.

`robot_ip_1` and `robot_ip_2` are required to be the empty string. The
generated collision geometry does not depend on them, and a committed artefact
never contains a network address.

The generator is fail-closed: it aborts rather than emit a volume it does not
fully understand — a `<geometry>` with other than one child, a collision shape
that is not a cylinder, sphere or box, a link whose collision count is not a
multiple of three, a triple that is not `[cylinder, sphere, sphere]`, a
`safety_distance` it cannot recover or whose call sites disagree, a radius whose
base is not one of the five the description uses, or a joint that is neither
revolute nor fixed.

Nine of the ten volumes per arm are *exactly* the capsule their three source
primitives compose into. `link8` is not: its two spheres are separated
perpendicular to its cylinder's axis, so no capsule reproduces it. It is covered
by a deliberately larger capsule of radius `0.0605`, whose minimal enclosing
radius is `sqrt(0.06² + 0.005²) = 0.060207972894` and whose recorded
`containment_margin` is therefore `0.000292027106`. The obvious answer, `0.06`,
under-covers by 0.21 mm — invisible in any picture, survives any smoke test, and
would make the checker report "clear" for a configuration in which the modelled
flange rim is outside its own collision volume. The test suite proves the
containment by sampling the source surfaces rather than asserting it.

Capsule endpoints are emitted in descending lexicographic order on `(z, y, x)`,
rounded to twelve decimals. A capsule is orientation-independent, so the order
would not matter if the file were not compared byte for byte — and it is.

---

## 5b. `mesh_bodies_v1.yaml` — the convex bodies

A second generated artefact, derived by `generate_mesh_bodies.py` from
`franka_description`'s own collision meshes and visual shells. It is **the
geometry the fence measures**: steps 2a, 2b, 3 and 4a of section 4 run on these
bodies, and `test_the_switch_changed_this.py` pins the switch against the
committed capsule baseline, so the change of geometry cannot be undone or
re-made quietly.

### What a body is

For each link the *metal proxy* is the union of the description's own collision
solid and its own visual shell:

```
M(L) = collision solid  ∪  visual-shell solid
```

and the bodies are built to **contain** that union, not to approximate it. Each
shell triangle is assigned **as a whole triangle** to the collision piece
nearest to it, and the body is the convex hull of that piece's vertices together
with the vertices of its assigned triangles. A convex hull contains the hull of
any subset of its generators, so every assigned triangle lies inside the body it
was assigned to; hence the whole shell surface lies inside the union of the
bodies. That is a proof, and it is what lets a clearance be reported with
**nothing subtracted from it** — there is no undercut term, because there is no
undercut.

Assignment is by triangle and **not by vertex**. The vertex-wise version splits
triangles that span two collision pieces, and the union then stops containing
the shell — measured at about a millimetre on `link5`, the one link that binds.

The collision meshes undercut the visual shell by up to 6.5026 mm on `link5`.
That figure is recorded per link as `shell_undercut_m` and is **used by
nothing**: it exists so that an asset change moves a number in a diff instead of
moving a clearance in silence.

### `link8` is different, and the difference is written down

`franka_description` ships **no `link8` mesh at all** — no visual, no collision.
`meshes/visual/` holds `link0.dae`…`link7.dae`, `hand.dae` and `finger.dae`; the
URDF's `link8` block carries three `<collision>` elements and no `<visual>`
element; `mj_dual.xml` gives `mj_left_link8` a site and the hand subtree with no
geom. So `link8_flange` is not derived from geometry anybody measured. It is the
convex hull of the URDF's own three primitives at **metal** radius — the
declared radius with the recovered `safety_distance` subtracted, the same
inflation-recovery discipline every capsule gets — sampled on a lattice inflated
by a pinned scale so the hull **circumscribes** the primitives.

The scale is not cosmetic. A convex hull of points sampled *on* a sphere lies
strictly **inside** that sphere, so the obvious construction produces a body
0.308 mm **smaller** than the geometry it claims to represent. That is optimism,
on the one body in the model with nothing behind it, touching six of the sixteen
enabled self pairs. The artefact therefore carries a **containment certificate**
checked in closed form: a convex polytope contains a convex set exactly when
every face plane satisfies `h_P(n) ≤ d`, and the support function of a union of
two spheres and a cylinder is closed form, so the check is over all directions
rather than at some resolution. Zero of 920 faces violated, minimum slack
+7.87e-07 m.

`body_source` distinguishes it from every mesh-derived body, and the written
reason names the one measurement that retires it: a caliper reading of the
flange boss with the arm powered down.

### The reader's three caps

`strictyaml`'s `MAXIMUM_MODEL_BYTES` (65 536), `MAXIMUM_YAML_DEPTH` (8) and
`MAXIMUM_YAML_SCALARS` (2 048) stay in force for the cell files. This artefact
carries 11 593 vertices — **34 779 numeric scalars**, seventeen times the scalar
cap — so a reader built on those constants would refuse it long before any byte
bound was reached. It therefore has its own three, named and measured:

| cap | value | measured |
| --- | ---: | --- |
| `MAXIMUM_MESH_BODIES_BYTES` | 786 432 | 545 223 bytes emitted |
| `MAXIMUM_MESH_BODIES_SCALARS` | 40 000 | 34 779 vertex scalars plus metadata |
| `MAXIMUM_MESH_BODIES_DEPTH` | 8 | maximum nesting 7 |

The **emission style** is part of the contract, not a preference: one
flow-sequence line per vertex at twelve decimals. A fully block-style emission
of the same numbers is about 300 KB larger, which is more than the whole margin,
and `test_bodies_regenerate_byte_for_byte` pins it.

### Load rules

- `mesh_bodies` and `mesh_bodies_sha256` are pinned in the cell file's
  `sources`, exactly like the link geometry. **A mismatch is a load failure and
  never a fallback to capsules** — falling back would mean an asset problem
  silently produces the looser fence.
- `metal_definition` must be `collision_union_visual_shell` and `assignment`
  must be `whole_triangle_nearest_piece`. Both are claims the file makes about
  how it was built, and both name what goes wrong when they are false.
- The flange's `recovered_safety_distance` must equal the link geometry's, its
  `lattice_scale` must be at or above the minimum containing scale, and its
  recorded certificate must show no violation.
- A body may not carry `inflation` or `coverage`. A mesh body contains the
  visual shell by construction; there is nothing to add back, and a key that
  says otherwise means somebody has reintroduced the model this artefact
  replaces.
- `link_geometry_v1.yaml`'s `source.generator_version` must be at least 2. A
  link geometry generated before the mesh work fails **by name** rather than by
  a digest mismatch whose message names only a file.
- The description digest the two artefacts record is compared and a difference
  is a **diagnostic, not a refusal**. The bodies are per-link, carry no arm
  prefix, and depend only on inputs the single-arm and dual descriptions share,
  so a single-arm session uses them as they are — but the provenance difference
  is said out loud rather than left to be inferred.

---

## 6. The API

```python
from franka_workspace_model.model import CellModel, default_cell_model_path

model = CellModel.load(default_cell_model_path(), profile='dual')

model.arm_ids()          # ('panda1', 'panda2')
model.allowed_volume()   # AllowedVolume(id, frame, x_min, ..., z_max)
model.urdf_sha256()      # for your own interlock against the running description
model.xacro_args()       # the argument set that digest was produced with
model.diagnostics()      # load-time facts; none of them is a defect
model.model_identity()   # (model_id, revision, sha256), as carried by every result

model.check_configuration(q, first_violation=False)   -> CheckResult
model.check_path(waypoints, first_violation=False)    -> CheckResult
model.check_jog(arm_id, q_now, joint_index, delta)    -> JogResult

canonical_urdf_text(rendered)   # xacro output, normalised the way the digest is
urdf_digest(rendered)           # sha256 of that; compare with model.urdf_sha256()

result_to_json(result)   # exactly the dataclass field names, floats at 6 decimals
```

`q` maps `arm_id` to seven joint positions in radians, ordered `joint1..joint7`,
for **every** arm the model declares — not just the one you are moving. A jog is
only safe relative to where the other arm actually is, so pass the other arm's
measured values; refusing to check is better than assuming a pose for it.

`default_cell_model_path()` returns the installed cell file, or `None` when
there is none, so a consumer never has to hard-code a filename from this
package. The stable install location is
`share/franka_workspace_model/cell/cell_model_v1.yaml`.

The accessor returns a path only when that path can actually be **loaded**,
which by the resolution rule under "Sources" means the description it was
derived from is reachable from it: a source checkout, or an install space built
with `colcon build --symlink-install`. A plain `colcon build` copies `cell/`
into the install space and leaves the description behind, and that copy cannot
be loaded at all — so the accessor withholds it rather than handing back a path
that is certain to raise. `None` therefore means "no cell model this consumer
can use", not "no file"; the two are distinguished by whether
`share/franka_workspace_model/cell/cell_model_v1.yaml` exists on disk.

**Concurrency.** A loaded `CellModel` is immutable: the parsed structure is
never mutated after load, and no check writes module-level state. All three
check methods are therefore safe to call concurrently from several threads on
one loaded model, with no lock.

### `check_jog` clamps, then reports

`delta` is reduced toward zero to the largest safe whole multiple of
`policy.max_joint_step_rad`. If some of the requested motion is safe, the result
is `allowed: true`, `clamped: true`, and a `q_target` short of what you asked
for, with `limiting` naming the contact that stopped it. If even the first step
is unsafe the result is `allowed: false`, `clamped: false`, `q_target` unchanged
and `limiting` set.

**A jog that cannot travel is refused, never approved.** That covers both ways
of getting there: a start pose that already violates, and a start pose that is
clear at swept margins whose very first step is not. An approved jog whose
`q_target` equals `q_now` would give a console a move it may command and that
changes nothing — clicks that read as a hung interface rather than as a fence.
A `delta` of exactly `0.0` is the one exception, because it has no first step:
it is a query about where the arm already is, and it answers `allowed: true`,
`clamped: false` when that pose is clear.

The jog is checked at swept margins, `margins.<kind> + margins.swept_path_extra`,
exactly as `check_path` is. `check_configuration` can therefore say a pose is
allowed while `check_jog` refuses to move from it, and that is the intended
ordering: the fence in front of a moving arm is the stricter one.

A user interface must show a clamped jog as clamped. Clamping lets an operator
creep toward a boundary one click at a time, and the mitigation for that is
visibility: every clamped click has to be visible, or creeping becomes silent.

### What this model does not fence

**The workspace model fences server-mediated motion only.** Where a system lets
an arm be driven by a node that publishes to the controller directly — a source
switch, an external planner, a teach pendant — the model is not in that path and
cannot be put into it. For such an arm the only surviving fence is the
controller's own per-joint box, which has no Cartesian, self-collision,
cross-arm or containment awareness whatsoever.

Three obligations follow, and they are binding:

1. no user-interface copy may describe this model as protecting the cell, the
   arms or the operator without qualifying it to server-mediated motion;
2. when an arm is driven externally, the model's status for that arm must read
   as *not applicable*, never as *active*;
3. this is a limitation of where the model sits, not a defect to be fixed by
   inserting it into someone else's publisher.

### The layering invariant

There are three enforcement layers, in strictly decreasing privilege: the
reviewed controller's per-joint box and slew limiter, inside the 1 kHz loop;
this model, on a non-real-time request path; and the physical stops and E-stop.

> **The workspace model may only ever reject. It may never widen, relax,
> override or substitute for any check the controller performs.** Every
> configuration this model permits must independently satisfy every check the
> controller makes.

Consequences, all of them binding on any consumer:

- the model is never linked into, imported by, or called from anything that runs
  inside the control loop; the core module imports no ROS at all, and that is
  asserted by a test rather than left as a convention;
- a model failure — file missing, hash mismatch, an unexpected exception —
  disables jogging; it never falls back to "allow";
- the model never writes or proposes a controller parameter, and never restates
  a joint limit numerically: it reads them from the policy file the controller
  itself is validated against;
- if the model and the controller ever disagree, the controller wins, because it
  runs afterwards and rejects independently. A disagreement is a bug report
  against this model, never a reason to loosen the controller.

---

## 7. What is deliberately absent

- **No rendering.** No marker array, no SVG. A picture is useful, not
  load-bearing, and a rendering subsystem inside a collision model is a module
  the layering invariant then has to be reasoned about.
- **No inverse kinematics, and no repair.** The model rejects candidates; it
  does not nudge joint values toward feasibility.
- **No payload or dynamics.** This is a geometric checker. Recording a mass in a
  field that looks like the checker uses it would be worse than omitting it.
- **No mesh-accurate geometry.** No collision mesh is wired into the
  description, the mesh assets that do exist have no established provenance for
  a safety path, and no mesh-distance library is installed. Where the built-in
  inflation over-approximates the real hardware, this model refuses poses that
  are physically clear. That is the intended direction of error.
- **No cross-validation against an independent collision library.** That test
  exists and is skipped explicitly, with its entry condition attached, rather
  than vanishing from the report.
