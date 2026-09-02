# The 2F-85 enclosing capsule

This file is the derivation of the collision volume that stands for a mounted
Robotiq 2F-85 in the workspace model. It is written to be finished from itself:
someone with the hardware mounted, a caliper and this page can complete it
without opening anything else.

Two numbers are still blank, and they are blank because a document cannot
supply them — they are measured on the bolted stack. Everything else is
computed here, from dimensions printed in the gripper's own manual, with the
page each one came from.

**Source.** *Robotiq 2F-85 & 2F-140 Instruction Manual*, revision **2018/05/23**
(title page © 2018 Robotiq Inc.). Page numbers below are PDF page numbers, and
in this region they coincide with the manual's own printed page numbers.

---

## 1. Symbols

Lengths are **millimetres** in the derivation and **metres** in the YAML. That
change of unit happens exactly once, at the paste-ready block in section 8, and
it is the single most likely place to make a mistake reading this file.

| Symbol | Meaning | Value | Where it comes from |
|---|---|---|---|
| `w` | maximum width — the finger-opening direction, `x` | **148.6 mm** | §6.2 Mechanical specifications table, 2-Finger 85 column, "Maximum width", page 135 |
| `h` | maximum height — the tool axis, `z` | **162.8 mm** | same table, "Maximum height", page 135; printed again on Fig. 6-2 (closed), page 120 |
| `d` | depth — the extent perpendicular to finger travel, `y` | **75.0 mm** | body diameter ⌀75, §6.1 **Fig. 6-1 bottom view, page 119** (section 2) |
| `r_body` | half-diagonal of the lateral rectangle | **83.227 mm** | derived: `0.5·sqrt(148.6² + 75.0²)` |
| `radius` | the capsule radius as shipped | **83.5 mm = 0.0835 m** | `r_body` rounded up to the next 0.5 mm |
| `t_c` | fitted coupling **stack height**, flange face → gripper base | **blank — measured on mounting day** | see section 3 |
| `r_cbl` | outermost radius of any cable or clamp riding on the flange | **blank — measured on mounting day** | see section 3 |

`w` and `h` are the manual's own *Maximum* figures. `d` was read off the
drawing; section 2 records what it beat.

---

## 2. The depth `d`, read rather than deferred

Fig. 6-1 ("2-Finger 85 general dimensions (opened)", page 119) carries three
views. Every candidate for the largest `y` extent, with the value printed
beside it:

| Candidate | Printed | View | Verdict |
|---|---|---|---|
| Body outer diameter | **⌀75** | bottom view (`x`–`y` plane), page 119 | **governs** |
| Body spigot / recess diameter | ⌀71h8 | same view | inside ⌀75 |
| Finger-stack width in `y` | **39** (over 22 and 14) | side view (`y` horizontal), page 119 | 36 mm inside ⌀75 |

So `d` = **75.0 mm**, governed by the ⌀75 body diameter.

**Why closing the fingers cannot increase it.** The fingers travel in the
`x`–`z` plane; the feature that sets `d` is the **body**, which does not move at
all. In the open pose — the pose in which the fingers reach furthest — the
mobile parts reach only 39 mm in `y`, 36 mm inside the governing 75. The
invariance of `d` over the stroke is therefore structural, and it is argued
rather than measured twice on purpose: Fig. 6-2 (page 120) is a **front view
only**. It prints 122.5, 162.8, 26.7 and 12.7 and carries no `y`-direction view,
so it cannot corroborate a depth, and anyone told to check `d` against it is
being asked for something the page does not contain.

**The figure's own caveats, quoted, because they are why the containment margin
is not decorative:** *"APPROXIMATE DIMENSIONS"*, *"HEIGHT AND WIDTH OF THE
FINGERS VARY WITH OPENING POSITION"*, *"UNITS IN MM"*.

---

## 3. The two blanks, and why they are blanks

**`t_c` — the assembled flange-face-to-gripper-base height.** This is a stack
height on the bolted assembly, measured with calipers. It is **not** a
coupling's catalogue thickness. Fig. 6-6 (page 124) prints **13.9 mm** as the
total thickness of the ISO 9409-1-50-4-M6 coupling `AGC-CPL-062-002`, with a
3 mm step on its robot side and a ⌀63 F8 recess; that coupling seats **into**
the gripper's ⌀71h8 recess, and Fig. 6-2's 162.8 dimension terminates on a
plate only about 3–4 mm thick as drawn. The catalogue figure and the
flange-to-gripper offset are different quantities. If the measurement happens
to land on 13.9 mm, that is a result, not a confirmation of an expectation.

**`r_cbl` — the cable standoff.** The outermost radius, from the tool axis, of
any cable or clamp riding on the flange. It is the only remaining blank that
can change `radius`, and only if it exceeds 83.227 mm — a cable standing more
than 83 mm off the tool axis, which would be a routing problem in its own
right.

`franka_robotiq/doc/MOUNTING.md` collects both, in its section 3 and its
mounting-day log table.

---

## 4. The construction

```
Declared enclosing solid  B = [-w/2, w/2] x [-d/2, d/2] x [z_a, z_b]
                              axis-aligned in <arm_id>_link8, centred on +z

r_body  = 0.5 * sqrt(w^2 + d^2)                # half-diagonal of the lateral rectangle
r_min   = max(r_body, r_cbl)                   # NOT r_body + r_cbl -- see below
radius  = ceil(r_min to the next 0.5 mm)       # a number a human can check
containment_margin = radius - r_min            # > 0 by construction, recorded not rounded away

a = (0, 0, z_a)   with  z_a = t_c
b = (0, 0, z_b)   with  z_b = z_a + h

evaluated with the printed dimensions:
r_body  = 0.5 * sqrt(148.6^2 + 75.0^2) = 0.5 * sqrt(27706.96) = 0.5 * 166.4541
        = 83.227 mm
radius  = 83.5 mm                              # = 0.0835 m
containment_margin = 83.5 - 83.227 = 0.273 mm  # = 0.000273 m
```

**The cable standoff competes; it does not stack.** `r_cbl` is defined as an
**absolute radius from the tool axis** — the same quantity `r_body` is — so the
two compete for the maximum rather than adding. Writing `r_body + r_cbl` would
declare a solid that contains the cable twice over and inflate the capsule for
no gain. The consequence is carried through section 7: the containment margin
is measured against the **union** of the body, the coupling and the cable, not
against the box alone.

`|b - a| = h = 162.8 mm`, comfortably above the degenerate-capsule floor of
1e-9 m, so `a != b` holds independently of both remaining blanks.

**What the shipped `radius` does and does not assume.** It is computed from two
printed dimensions and nothing else. It is provisional in exactly one
direction: if the measured `r_cbl` exceeds 83.227 mm, `r_min` rises and
`radius` must be recomputed. Section 9 says so in one line.

---

## 5. The union over the whole stroke

Fig. 6-1 (opened, page 119) prints the overall width as **148.6** and the
overall height in that pose as 148.9. Fig. 6-2 (closed, page 120) prints the
width as **122.5** and the height as **162.8**. So the **maximum width occurs
in the open pose** and the **maximum height in the closed pose**, and §6.2's
table reports each of them as a *Maximum*.

The box `B` uses both maxima **simultaneously**. That is a strict superset of
either extreme pose, and therefore of every pose between them. This argument
has to be made rather than assumed precisely because the figure says height and
width vary with opening position.

---

## 6. The coupling is inside — through the lower cap, not through `[z_a, z_b]`

A reader who sees `z_a = t_c` will reasonably conclude the coupling was left
out of the volume. It was not.

A capsule is a swept sphere, so the solid includes the hemisphere of radius
`radius` **below** `a`. The coupling occupies

```
C = { rho <= r_c , 0 <= z <= z_a }        with r_c = 37.5 mm  (⌀75/2, Fig. 6-6, page 124)
```

and every point of `C` lies inside the capsule if and only if

```
sqrt(r_c^2 + z_a^2) <= radius
```

With the documented candidate `t_c = 13.9 mm`:
`sqrt(37.5² + 13.9²) = sqrt(1599.46) = 39.99 mm`, against `radius = 83.5 mm` —
a factor of two of headroom. The inequality only becomes tight at
`t_c = sqrt(83.5² − 37.5²) = 74.6 mm`, which no coupling approaches.

**The check is still written into section 9 with a blank for the day's `t_c`
and `r_c`, and it must be re-evaluated rather than assumed.** Printing the
headroom is what makes "re-evaluate" a real instruction instead of a ritual.

---

## 7. The frame, the clocking, and the containment proof

### The frame

`a` and `b` lie on the `<arm_id>_link8` `+z` axis, with the origin on the
flange face and `+z` pointing outward, away from the arm. Corroboration from
inside this repository, so the claim is checkable rather than asserted:
`link8` is the flange link, and its own collision primitives sit at
`z = -0.02 … -0.03` — *behind* the frame origin — which is consistent with the
origin lying on the flange face and `+z` pointing away from the arm.

Because `B` is centred on that axis and the capsule is a solid of revolution
about it, **a rotation about `+z` maps the capsule to itself.** The clocking
error that a derivation of this kind usually has to worry about is therefore
undetectable *and harmless* for this geometry.

**It is not harmless for the load declaration, and nobody should carry
"clocking doesn't matter" out of this file.** The manual's inertia matrix has
`I_xx = 2768` against `I_yy = 3149 kg·mm²`, so a lateral rotation of the mount
redistributes the two lateral entries. `franka_robotiq/doc/MOUNTING.md` handles
it, in its section 6.

Note that this says **a rotation**, not *a 90° rotation*. `link8`'s own
collision primitives sit at `xyz="0.0424 0.0424 …"` — a 45° diagonal — so the
relationship between that frame's axes and the flange bolt pattern is not
established in this phase. A mount that is neither 0° nor 90° would make
neither the exact matrix nor its swapped variant correct. The runbook asks for
the clocking **in degrees** for that reason.

### Containment, proved

*P1 — the box.* For any `p = (px, py, pz)` in `B`, `pz` lies in `[z_a, z_b]`,
so the nearest point of the segment `ab` is at the same `z` and the distance is
exactly `sqrt(px² + py²) <= sqrt((w/2)² + (d/2)²) = r_body <= r_min <= radius`.
With the read dimensions: `sqrt(74.3² + 37.5²) = 83.227 mm <= 83.5`. Each of
the eight corners attains `r_body` exactly, so `r_body` is also the *minimal*
radius containing `B` on this segment — which is what makes
`containment_margin` a real quantity rather than a shrug.

*P2 — the coupling.* The inequality of section 6.

*P3 — the stroke.* The argument of section 5.

Hence `containment: conservative` with `containment_margin > 0`.

### What the margin is measured against

`franka_workspace_model/doc/CONTRACT.md`, section "The switchable end
effector", defines the containment margin as *the derived radius minus the
minimal radius containing the union of the source `<collision>` primitives*.
This volume has no URDF source, so the **declared solid takes that role**; and
because `r_min = max(r_body, r_cbl)`, the margin is the margin against the
**union of the box `B`, the coupling cylinder `C` and the cable**, not against
`B` alone as P1 taken by itself would suggest. Both sentences are here so that
nobody later reads the margin against a definition it was not computed under.

### Sampling this volume in a surface test

A surface test written against source `<collision>` primitives has nothing to
sample here. The adaptation: sample at least 2000 points on the surface of the
declared box `B` **and** on the coupling cylinder `C`, and assert each lies
inside the capsule to 1e-9.

---

## 8. No double counting — and what the end caps cost

`radius` carries **no** safety distance. For a pair of this capsule against a
URDF-derived link volume, the physical clearance is `margin + 0.03`, not
`margin + 0.06`, because only one of the two bodies carries built-in inflation.

Two consequences follow, and they pull in opposite directions. Both are
printed, in this order, because either one alone sends a margin-setter the
wrong way.

**One: a pair of two end-effector capsules buys `margin + 0.00`.** Two grippers
meeting at the cell midplane is the single most likely contact in a dual cell,
and neither body carries inflation. Taken alone, that reads as "less clearance
than you think".

**Two, and it dominates: the end caps.** A capsule is a swept sphere, so it
extends `radius` beyond each endpoint:

```
lower cap reaches  z_a - radius = t_c - 83.5 mm    (about -70 mm with t_c = 13.9:
                                                    behind the flange face, into the wrist)
upper cap reaches  z_b + radius = t_c + 246.3 mm   (about 83.5 mm past the real fingertips)
```

So the two-gripper midplane pair reports contact roughly **167 mm** — two
83.5 mm caps — before the fingertips actually meet. That is the dominant
practical effect in this cell, and it pushes the margin recommendation *down*,
not up. The caps are also why `containment: conservative` is honest: the
declared solid genuinely contains the hardware, with room to spare.

---

## 9. The paste-ready block, and how to finish it

```yaml
    end_effector:
      present: false                 # refused while any volume is to_be_derived
      profile: robotiq_2f85
      volumes:
        - id: robotiq_2f85_v0
          kind: capsule
          a: [0.0, 0.0, null]        # z_a = t_c  -- measured on the mounted stack (TBC-6)
          b: [0.0, 0.0, null]        # z_b = z_a + 0.1628
          # radius = 0.5*sqrt(0.1486^2 + 0.075^2) = 0.083227, rounded up to 0.0005
          # d = 0.075 m: body diameter 75 mm, Fig. 6-1 bottom view, PDF page 119
          radius: 0.0835
          containment: conservative
          containment_margin: 0.000273   # radius - r_min, r_min = r_body = 0.083227
          derivation_status: to_be_derived
          provenance: "Robotiq 2F-85 & 2F-140 Instruction Manual, revision 2018/05/23:
            section 6.2 Mechanical specifications table, 2-Finger 85 column (Maximum height
            162.8 mm, Maximum width 148.6 mm); section 6.1 Fig. 6-1 (opened), PDF page 119,
            bottom view, body diameter 75 mm for the depth; section 6.1.1 Fig. 6-6 for the
            coupling. Enclosing-capsule derivation and containment proof:
            franka_robotiq/doc/CAPSULE.md"
```

Two things about that block.

* **`radius` is a number because the desk could settle it.** Shipping it blank
  would have handed the next reader an unmeasured `null` for the one value
  derivable from documents alone. The printed value and page of `d` sit on the
  adjacent comment lines so the number is checkable without leaving the file.
* **`a` and `b` stay `null`, and that is the safety property.** A `null` in a
  coordinate fails the schema before the "no `present: true` while any volume
  is `to_be_derived`" rule ever runs, so the file refuses twice.
  `derivation_status: to_be_derived` now rests on the coupling stack height and
  the cable standoff alone, and it still refuses `present: true`. Nothing about
  the safety property changed when the depth closed.

### The fill-in procedure

1. Measure `t_c` — the **assembled** flange-face-to-gripper-base height on the
   bolted stack, not the coupling's catalogue thickness. Set
   `a: [0.0, 0.0, t_c]` in metres.
2. Set `b: [0.0, 0.0, t_c + 0.1628]`.
3. Measure `r_cbl`, the outermost radius of any cable or clamp riding on the
   flange.
4. **Only if `r_cbl > 0.083227`**: recompute `r_min = r_cbl`,
   `radius = ceil(r_min` to the next `0.0005)` and
   `containment_margin = radius - r_min`, and replace both numbers. Otherwise
   the shipped `radius` and `containment_margin` stand unchanged — leave them
   alone rather than editing a correct number to prove the step was done.
5. Re-run the coupling inequality `sqrt(r_c² + t_c²) <= radius` with the day's
   `r_c` and `t_c`.
6. Set `derivation_status: derived`.
7. Then, and only then, flip `present: true`.

### Why `z_b` is safe before `t_c` is known

`h = 162.8 mm` was measured with a coupling fitted (§6.2's own footnote names
`AGC-CPL-062-002` and fingertip `AGC-TIP-204-002`). Two readings of its datum
are possible:

* datum = the coupling's **robot-side** face ⇒ the true top is `h`, and
  `z_b = t_c + h` over-states by `t_c`;
* datum = the coupling's **gripper-side** face ⇒ the true top is `t_c + h`,
  exactly `z_b`.

**In both readings `z_b` is at or above the true top**, so the formula is safe
whichever it is. Zooming Fig. 6-2's 162.8 extension line does not settle it:
the line lands on the underside of a thin plate at the base of the body, with
the indexing pin excluded, and no full coupling is drawn standing proud of the
gripper base. That is the same observation that makes `t_c` an assembled stack
height rather than a catalogue number.

### `source_elements`

End-effector volumes carry no `source_elements` key: there is no URDF source to
partition, and the workspace model's own worked example omits it on this
volume. A loader that requires the key everywhere must exempt end-effector
volumes explicitly rather than by accident.

---

## 10. What is still open, stated plainly

* `t_c` and `r_cbl` are measured on mounting day. Until then
  `derivation_status` stays `to_be_derived` and `present: true` is refused.
  This is the design working, not a gap.
* The clocking of the gripper relative to `<arm_id>_link8`'s axes is not
  established in this phase. It changes nothing here — the capsule is a solid
  of revolution — and it matters for the load declaration, which
  `franka_robotiq/doc/MOUNTING.md` handles.
