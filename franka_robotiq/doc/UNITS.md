# Units, and what the numbers mean

Everything an operator or a caller sees is in **millimetres**, **newtons** and
**millimetres per second**. The gripper itself speaks in counts, 0 to 255, on
every axis it has. All the conversion happens in one module,
`franka_robotiq/units.py`, and nowhere else.

This file is about which of those conversions are documented by the
manufacturer and which are honest guesses — because two of them are guesses,
and a caller who does not know that will read more precision into a force
setting than is there.

---

## 1. The conversion table

| Conversion | Formula | Status |
|---|---|---|
| count → width | `width_mm = 85.0 * (255 - count) / 255` | **DOCUMENTED-LINEAR.** The manual states count `0x00` = fully open, `0xFF` = fully closed, *quasi-linear* between them, and that activation re-calibrates the endpoints to whatever fingertips are fitted. This is the mapping the manual describes. |
| width → count | `count = round(255 * (85.0 - width_mm) / 85.0)`, clamped to 0…255 | same |
| speed → count | `count = round(255 * (speed_mm_s - 20.0) / (150.0 - 20.0))`, clamped | **INTERPOLATION, NOT DOCUMENTED.** The manual gives only the endpoints — count `0x00` is minimum speed, `0xFF` maximum — and a range of **20.0 to 150.0 mm/s**. Linearity between them is *assumed*. |
| force → count | `count = round(255 * (force_n - 20.0) / (235.0 - 20.0))`, clamped | **INTERPOLATION, NOT DOCUMENTED.** Endpoints **20.0 to 235.0 N** from the product sheet. |
| motor current | `current_ma ≈ 10 × count` | The manual's own approximation, and it says "approximate". |

**The two interpolation rows are not manufacturer curves.** They are straight
lines drawn between two documented endpoints. Nobody has published what force a
count of 128 actually produces, and this package does not pretend to know.

For what a given force setting really delivers, the authority is the manual's
own **measured force table** in its force-control section: 25–220 N for steel
or aluminium fingertips on a hard payload, 25–155 N on 40 A silicone, 25–115 N
on soft neoprene and polyurethane. Force repeatability is **±10 %**. Treat a
requested value in newtons as a dial setting with a documented range, not as a
calibrated force.

---

## 2. The 0.4 mm question

The manual and the product sheet both print a position resolution of **0.4 mm**
per count. That cannot be the scale factor: 255 counts × 0.4 mm is 102 mm, and
the stroke is 85 mm.

The reading: 0.4 mm is a nominal fingertip-resolution figure, quoted as the
increment produced by a one-bit change, not the count-to-width scale. The
driver uses the stroke over the count range instead — 85 mm / 255 counts =
**0.333 mm per count** — which is the mapping the endpoints actually describe.

---

## 3. Half-width, everywhere

> **`~/gripper_action` is half-width in metres — in the goal, in the feedback
> *and* in the result.** `0.0` is fully closed, `0.0425` is fully open. That is
> one finger's opening, not the gap between the fingers.
>
> `franka_gripper` interprets its *goal* as half-width but reports its
> *feedback and result* as full width. That asymmetry is internal to that
> package; `franka_robotiq` does not copy it. A goal → result round trip is
> consistent here, and it is not there.

The same convention holds on `~/joint_states`: two finger joints, each carrying
half the width, in metres. That matches the shape `franka_gripper` publishes,
so code written against the Franka Hand consumes it unchanged.

`~/status` is the exception, and it is deliberate: `width_mm` there is the
**full** opening in millimetres, because that is the number an operator reads
off a caliper.

| Where | Quantity | Unit |
|---|---|---|
| `~/gripper_action` goal, feedback, result `position` | half-width | metres |
| `~/gripper_action` `max_effort`, `effort` | commanded force | newtons |
| `~/joint_states` `position`, both joints | half-width | metres |
| `~/status` `width_mm`, `requested_width_mm` | full width | millimetres |
| configuration and ROS parameters | width, speed, force | mm, mm/s, N |

---

## 4. Why `velocity` and `effort` are zero on `~/joint_states`

Because the protocol reports neither.

There is no velocity field on the wire at all. There is a motor current, but a
motor current is not a fingertip force — it is the current the motor is drawing
against whatever the mechanism is doing, and converting it into an `effort`
figure would put a fabricated number into a field consumers trust.

The real current is on `~/status` as `current_ma`, labelled as what it is: the
manual's own approximation of roughly ten times the raw count.

`effort` on the action's feedback and result is different again — it is the
**commanded** force in newtons, the value that produced the count that was
sent, never a measured one.

---

## 5. The `at_position` trap

`~/status` reports object detection as one of five words:

| Value | Meaning |
|---|---|
| `unknown` | the gripper is not executing a motion, so its detection bits are meaningless and are reported as meaningless rather than as "nothing" |
| `none` | moving toward the target, nothing hit yet |
| `closed_on_object` | stopped on contact while closing, short of the request — the normal "gripped it" outcome |
| `opened_on_object` | stopped on contact while opening, short of the request |
| `at_position` | at the requested position |

**`at_position` after a close does not prove the object is still held.** It
means "the fingers reached what you asked for", and that is equally true when
nothing was ever there and when the object was dropped on the way.

The manual's own caution, in substance: a thin object in a fingertip grip can
be held successfully **without** detection firing, so `at_position` is
sufficient to proceed in such applications; and its own tip is that checking
the finger position **together with** the detection state is more reliable than
either alone. Read `width_mm` and `object` together.

The node never reduces this to a `holding: true/false` boolean, because the
protocol cannot support one and a boolean would be believed.

---

## 6. Services this package deliberately does not have

There is no `~/homing`, no `~/move` and no `~/grasp`.

`franka_gripper` has them because libfranka has them. The Robotiq protocol has
exactly **one** motion primitive — go to a requested position at a requested
speed and force cap — and inventing three names for one primitive would be
three fake knobs. What exists instead is `~/gripper_action` for motion,
`~/open` and `~/close` for the two configured positions, `~/stop` to hold where
the fingers are, and `~/reactivate` for the one operation the protocol really
does have as a separate thing: an activation cycle.

---

## 7. One note on the emulator's fault model

Running with the emulator (`use_fake:=true`) is the intended way to try the
whole surface with no hardware, and its protocol behaviour is held to the
manual. Its **fault** behaviour is partly an inference, and that is worth
knowing before you take it for documented behaviour:

* it latches fault `0x05` / `0x07` in response to a motion request made before
  activation is complete, and
* it runs the auto-release pair `0x0B` → `0x0F`.

The manual documents those codes as *meanings* only. It never states the
latching trigger, and it never states the transition. A real gripper that
behaves differently is a **finding to record** — step 8.11 of
`franka_robotiq/doc/MOUNTING.md` is where it gets recorded — and not a bug in
either the emulator or the gripper.
