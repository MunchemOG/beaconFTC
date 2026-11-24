# 07 — FTC legality and wiring (2026-27 BIOBUZZ manual, Section 12; re-check every season)

Source: https://ftc-resources.firstinspires.org/ftc/game/manual-12 (TU08/TU09, Nov 2026).
Deliverables: `RULES.md` (this table, cleaned) and `WIRING.md` with a printable diagram.

| Topic | Rule (quote/paraphrase) | How we comply |
| --- | --- | --- |
| Controller | R701: robots are controlled by one ROBOT CONTROLLER: (A) a REV Control Hub, or (B) a smartphone Android device connected to a REV Expansion Hub; plus (C) no more than one additional Expansion Hub. | Phone → Expansion Hub #1 (USB) + Expansion Hub #2 (RS-485). |
| Phone connection | R703: a smartphone used as RC must connect to the Expansion Hub using USB. | USB-C OTG → USB-A → Mini-B. |
| Which phones | No approved-device list any more; "legal but unsupported", team is responsible for its own testing. | Galaxy S22 (or any ARCore-certified phone). |
| Phone radios | R711: airplane mode on, Wi-Fi on, Bluetooth off. | Enforce at inspection checklist; the RC app's self-inspect shows it. |
| RC name | R705: RC should be named `<team>-RC`. | Set the phone's Wi-Fi Direct name. |
| Actuators | R503: total of 8 motors and 8 servos. | Two Expansion Hubs give 8 motor + 12 servo ports; use ≤ 8 servos. |
| Vision on the phone | The RC's own camera has always been usable (Vuforia/TFOD era); R707/R708 govern USB vision devices only. | VIO and AprilTags on the phone camera. |
| Coprocessor rule | R702: no modifying coprocessor software except SDK-supported programmable vision coprocessors. | Not applicable — the phone is the controller, not a coprocessor. |
| Extra phone as a sensor | Not allowed: it is neither the RC nor a permitted USB device. | Exactly one phone, and it is the RC. |
| Power | R602/R613: hubs powered from the robot battery via approved regulation; phone on its own battery like any RC phone. | Both hubs on XT30 from the main switch. |
| Lights | R202 F/J: no bright targeting lights left on; no flashing > 5 Hz. | None used. |

## Wiring (for `WIRING.md`)

```
 [Phone (RC)] --USB-C OTG--> [USB-A to Mini-B] --> [Expansion Hub #1 (addr 2)]
                                                        |  RS-485 (JST 2-wire)
                                                        v
                                                   [Expansion Hub #2 (addr 3)]
 [12V battery] -> [main switch] -> XT30 -> Hub #1
                                   XT30 -> Hub #2
```
- Change Hub #2's address with the REV Hardware Client before first use.
- Strain-relieve the phone USB cable at both ends; a locking OTG adapter + zip-tie anchor.
- Phone mount: rigid, near centre of rotation, rear camera clear, screen visible for inspection.
- Keep a printed copy of this table and the wiring diagram on the robot cart.

Not legal advice; the Head Referee and LRI decide at the event. If in doubt, ask the official Q&A
before the first event.
