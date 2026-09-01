# Binding a gripper to its adapter

Each gripper is reached over a USB-RS485 adapter. This file is about naming the
right one, and about what the software does when the name is wrong.

---

## 1. Why not `/dev/ttyUSB0`

`/dev/ttyUSB0` and `/dev/ttyUSB1` are handed out in probe order. Which adapter
gets which number depends on which one the kernel enumerated first, and that
changes across reboots, across replugs, and sometimes across nothing at all.

With two identical adapters on a two-arm cell, that is not a nuisance. It is a
**wrong-arm command**: you tell panda 1's gripper to close, and panda 2's
gripper closes, on whatever is between its fingers.

So the rule is:

> **A gripper is identified by its adapter's stable device name, and by nothing
> else. The driver opens exactly the configured name. It never scans, never
> picks "the only one present", and never falls back to `/dev/ttyUSB*`.**

---

## 2. Finding the names

Three commands. The first is the one you will use.

```bash
# the stable names, one symlink per adapter
ls -l /dev/serial/by-id/

# everything udev knows about one device, including its serial
udevadm info -q property -n /dev/ttyUSB0

# the USB vendor and product ids, for the udev rule in section 6
lsusb
```

**Plug them in one at a time.** Plug in the first adapter, run
`ls -l /dev/serial/by-id/`, write the new name down against the arm whose cable
it is. Then plug in the second and repeat. Both at once and you are guessing
which name belongs to which arm — and a guess here is exactly the failure this
whole file exists to prevent.

A by-id name looks like this:

```
usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0
```

`D3091K4T` is the adapter's USB serial number. That is the part that makes the
name stable, and it is the part the anti-swap check in section 5 reads back.

---

## 3. `serial_id` or `usb_path`

Exactly one of the two must be set for an enabled gripper. Both set, or neither
set, is a startup error with a message that says so.

| Key | What it names | Survives | Does not survive |
|---|---|---|---|
| `serial_id` | a basename under `/dev/serial/by-id/` | re-cabling to a different USB port | replacing the adapter |
| `usb_path` | a basename under `/dev/serial/by-path/` | replacing the adapter | re-cabling to a different USB port |

The two trade off in opposite directions, which is why the configuration says
which one is in force rather than the driver choosing.

**Prefer `serial_id`.** Use `usb_path` only when the adapter reports **no
unique USB serial** — some FTDI variants with a blank EEPROM serial, and CH340
parts, do exactly that, and two such adapters produce two identical or
ambiguous by-id names. When that happens the by-id name cannot tell the two
apart, so the binding falls back to the physical USB port. **Then physically
label the two ports**, with a marker, on the machine. The binding is now a fact
about a socket, and nothing in software can recover it if somebody swaps the
cables.

Two arms, two entries. In `~/.config/franka_web/config.yaml`:

```yaml
grippers:
  panda1:
    enabled: true
    serial_id: "usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0"
  panda2:
    enabled: true
    serial_id: "usb-FTDI_FT230X_Basic_UART_D3091K4W-if00-port0"
```

The same two values are `grippers.panda1.serial_id` and
`grippers.panda2.serial_id`; their by-path counterparts are
`grippers.panda1.usb_path` and `grippers.panda2.usb_path`.

On the standalone path they are launch arguments instead — `serial_id:=` for a
single arm, `panda1_serial_id:=` and `panda2_serial_id:=` for both.

---

## 4. The refusals, word for word

These are the messages the software actually prints. They are reproduced here
so that an operator who meets one can find it in this file and match it word
for word.

**Rule 1 — the configured adapter is not there.** The node starts, publishes
`link: down`, and logs once. It lists what *is* present, so your next action is
a copy-paste, and it never opens the one it found:

```refusal
panda1: no gripper at /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0
Adapters present now:
  usb-FTDI_FT230X_Basic_UART_D3091K4W-if00-port0
If the gripper was replaced, put the new name in grippers.panda1.serial_id.
See franka_robotiq/doc/SERIAL_BINDING.md.
```

**Rule 2 — both arms name the same adapter.** This one refuses to start rather
than warning, because a shared binding means one arm's gripper is silently
unbound while the other is driven by two owners:

```refusal
grippers.panda1.serial_id and grippers.panda2.serial_id are the same adapter
(usb-FTDI_...-if00-port0). One adapter cannot drive two grippers. Give each arm the
serial of its own adapter; see franka_robotiq/doc/SERIAL_BINDING.md.
```

**Rule 3 — the symlink lies.** After opening the port, the driver reads the
adapter's USB serial back out of sysfs and compares it against the serial
embedded in the configured by-id name. A mismatch closes the port immediately
and stays down:

```refusal
panda1: the adapter at that path reports serial D3091K4W, but panda1 is bound to
D3091K4T. Refusing to drive panda1's gripper through panda2's adapter. Nothing was
commanded.
```

**The port opened but nothing answered.** Not an anti-swap refusal — this is a
gripper that was reconfigured away from its factory serial settings:

```refusal
panda1: the adapter opened but the gripper never answered. franka_robotiq speaks
Modbus RTU at 115200 8N1 to slave ID 9, which is Robotiq's factory setting. If this
gripper was reconfigured with Robotiq's User Interface, set it back.
See franka_robotiq/doc/SERIAL_BINDING.md.
```

---

## 5. Where each refusal comes from, and the one gap

So that a bug report lands in the right place:

| Rule | Produced | When |
|---|---|---|
| 1 — the name is absent | at configuration time, in the package's discovery module | every attempt to resolve the name |
| 2 — both arms name one adapter | at configuration time, in the same module, and again in the web server's config validation | startup |
| 3 — the reported serial disagrees | in the gripper node, on **every** connect and reconnect | after the port opens, before anything is written |

Rule 3 is the belt-and-braces half. `/dev/serial/by-id` is built by udev, and a
stale or hand-made symlink can lie; the sysfs read catches that. It is
**best-effort**: when the serial is unavailable — a by-path binding, or an
adapter with no serial at all — the node logs one line saying that this check
is unavailable and that the binding rests on the physical port alone. It does
not pretend to have verified something it did not.

**And here is what no rule catches.** Giving one arm the *other arm's real
by-id name* is refused by nothing. Both names are real adapters; the serial in
the name matches the serial in sysfs, so rule 3 passes. Rule 1 does not fire
because the path exists. Rule 2 fires only when both arms are configured, which
a single-arm launch never is.

The only thing that catches it is a human: send a goal to one arm and **watch
which gripper moves**. That is step 8.10 of `franka_robotiq/doc/MOUNTING.md`,
and it is done with your eyes on the hardware, not on the terminal. A
documented gap beats a mis-asserted one.

---

## 6. The udev rule

`udev/99-franka-robotiq.rules`, installed with the package, does two things and
neither is a rename:

* it grants the `dialout` group access to the adapter — Ubuntu's default for
  `ttyUSB`, restated so the file explains itself;
* it tells ModemManager to leave the adapter alone. ModemManager probes a fresh
  `ttyUSB` with AT commands for several seconds, and those bytes land on the
  gripper's Modbus link.

It creates **no** `/dev/robotiq_panda1` alias, and that is deliberate: a second
naming system can only disagree with the first. `/dev/serial/by-id` already
exists, is created by the distribution's own rules, and is what the
configuration names.

Installing it:

```bash
sudo cp "$(ros2 pkg prefix franka_robotiq)/share/franka_robotiq/udev/99-franka-robotiq.rules" \
        /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The rule is scoped by USB vendor id. **Check yours with `lsusb` first** — the
file says which id it assumes and which two lines to change. It is optional
convenience, not a requirement: without it the driver still works, provided
your user can open the device.

---

## 7. Serial parameters are pinned, not configured

115200 baud, 8 data bits, 1 stop bit, no parity, slave ID 9. These are
Robotiq's factory settings and this package treats them as constants, not as
configuration keys. A knob whose only correct value is the factory default is a
fake knob, and offering one would invite somebody to turn it.

If a gripper arrives reconfigured, the symptom is a link that opens and never
answers, and section 4's last message says exactly that.

---

## 8. Permissions

The adapter is a `ttyUSB` device, so your user needs to be in the `dialout`
group:

```bash
groups                              # is dialout in the list?
sudo usermod -aG dialout $USER      # if not
```

Then **log out and back in** — a group change does not reach a session that is
already running. Until then the symptom is a permission error on open, which
the node reports as a link that will not come up.
