# Beacon: FTC legality notes for inspection (2026-27 BIOBUZZ season)

Audit pass 2: 2026-11-17 (pass 1 was 2026-11-16, for a phone Robot Controller). Not legal advice.
**The Head REFEREE and the Lead Robot Inspector (LRI) decide at the event.** If in doubt, ask the
official Q&A before your first event.

## Do these before inspection

1. **Use a Limelight 3A (LL_3A), not a Limelight 4 or 3G.** R702 Table 12-9 lists the Limelight 3A
   only. The manual names the Limelight 3G as prohibited. The Limelight 4 is not in the table. If
   the team owns a Limelight 4, do not bring it to inspection without a written Q&A answer.
2. **Change the Control Hub's Wi-Fi password to a non-default one** and keep Bluetooth disabled
   (R711 A, C). Show the self-inspect screen with no red X icons.
3. **Name the Control Hub `<team number>-RC`** (R705).
4. **Plug the Limelight into the Control Hub's blue USB 3.0 port**, with no battery pack or other
   outside power source feeding it (R602, R613 A).
5. **Keep factory firmware on the Pinpoint.** Update it only with goBILDA-provided firmware (R706 F,
   R702 examples). Never flash your own.
6. **Do all Limelight web-interface work with a laptop plugged straight into the Limelight, not
   over the robot's Wi-Fi.** No laptop on the Control Hub Wi-Fi during a match (R704 C). No live
   video or log streaming from the robot (R704 D).
7. **Place the robot, then press INIT, then leave it still** until the Pinpoint LED is green
   (G304 H; Pinpoint calibration). See the G304 row.
8. **Put the Q&A questions below to the official Q&A** and print the answers.

## How to read this file

Every row carries one of three marks.

- **Verified**: the rule text was read in this audit and the link is in the row.
- **Corrected**: verified, and it differs from `beacon-spec/specs/07-RULES.md` or from pass 1 of
  this file. The row says what changed.
- **Unverified**: the text or fact could not be reached. The claim is carried and labelled.

Limits that remain:
- The manual and Team Updates were reachable. Rows citing a rule are Verified or Corrected.
- Hardware facts (Pinpoint, Limelight, REV) come from the manufacturers' documents, not the
  manual. They are marked "(docs)" and are Verified only as far as the document says. Nothing was
  tested on a robot.
- The official Q&A site is a JavaScript application and could not be searched. Every existing Q&A
  answer on these topics is **unchecked**. Search it yourself.
- Rule numbers change between seasons. Re-check every season and after every Team Update. As of
  2026-11-17 the latest is TU09; the Competition Manual HTML was last modified 13 Nov 2026.

### What was read

| Item | Version | Where |
| --- | --- | --- |
| Competition Manual, full HTML | TU09 (12 Nov 2026), `BIOBUZZ Competition Manual - TU09.htm` | https://ftc-resources.firstinspires.org/ftc/game/cm-html |
| Section 12 (ROBOT Construction Rules), PDF | TU08 (5 Nov 2026). R701, R703 and R711 compared with the TU09 HTML: identical. | https://ftc-resources.firstinspires.org/ftc/game/manual-12 |
| Team Update 09 (12 Nov 2026) | Changes only G411 and a field guide. Nothing in Section 12 or 5. | https://ftc-resources.firstinspires.org/ftc/game/tu-latest |
| Combined Team Updates | TU00 to TU09. Section 12 notes cover R501, R503 (servos cut to 8), R801, R101, R105, R201. Nothing on R7xx. | https://ftc-resources.firstinspires.org/ftc/game/tu-combined |
| Inspection Checklist V26-27.3 (29 Oct 2026) | Official checklist | https://ftc-resources.firstinspires.org/ftc/event/inspection-check |
| Limelight 3A quick-start (docs) | Read 2026-11-17 | https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a |
| Limelight FTC programming guide (docs) | Read 2026-11-17 | https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming |
| goBILDA Pinpoint 3110-0002-0001 User Guide (docs) | 14 pages, read 2026-11-17 | https://servocity.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf |
| REV Control Hub ports (FTC docs) | Read 2026-11-17 | https://ftc-docs.firstinspires.org/control_hard_compon/rc_components/hub/ports/ch-ports.html |
| FTC SDK 12.0 in this repository | `GoBildaPinpointDriver` and `Limelight3A` classes found in `Hardware-12.0.0`; samples `SensorGoBildaPinpoint.java` and `SensorLimelight3A.java` in `FtcRobotController` | local |

The season is **BIOBUZZ presented by RTX**.

## What Beacon is, in inspection terms (this build)

- A REV Control Hub (REV-31-1595) is the ROBOT CONTROLLER. Optionally one REV Expansion Hub is
  added. There is no phone on the robot.
- A goBILDA Pinpoint Odometry Computer (3110-0002-0001) with two odometry pods and its own IMU is
  connected to a Control Hub I2C port. It reports the robot's position continuously.
- A Limelight 3A (LL_3A) is connected to the Control Hub's USB 3.0 port and is dedicated to
  Beacon. Its camera frames go over USB to the Control Hub, where Beacon matches them against the
  BIOBUZZ field geometry (hive, perimeter, floor) to correct the Pinpoint. **No AprilTags are used.**
  (Changed 2026-11-17 from the AprilTag/MegaTag2 plan; see DECISIONS.md.)
- The only software on the robot is the team's Robot Controller app (TeamCode, including Beacon's
  native C++ library inside the app) on the Control Hub, plus the Limelight's own stock firmware
  and pipelines. No ARCore.
- Logs go to the Control Hub's own storage and are read after the match. Nothing is streamed.

## Verified rule table (Control Hub, Pinpoint, Limelight 3A)

| Topic | Rule | Quoted wording (short) | How Beacon complies | Mark and source |
| --- | --- | --- | --- | --- |
| Controller | R701 | "ROBOTS must be controlled via 1 programmable ROBOT CONTROLLER" made of "a REV Control Hub (REV-31-1595), or a smartphone Android device connected to a REV Expansion Hub". "no more than one additional REV Expansion Hub (REV-31-1153)". | A Control Hub, and at most one Expansion Hub. The checklist: "Maximum one REV Control Hub and one optional REV Expansion Hub". | **Corrected.** Pass 1 used option B (phone). Option A applies now. Manual section 12.7, R701: https://ftc-resources.firstinspires.org/ftc/game/cm-html ; checklist, Control System item 1 |
| Supported device | R701 (second paragraph) | "the REV Control Hub is the only officially supported ROBOT CONTROLLER device." | The Control Hub is the officially supported device. No compatibility testing burden beyond normal. | **Corrected.** Pass 1 carried the "unsupported smartphone, team tests it" wording. That no longer applies. Same source. |
| Limelight 3A is allowed | R702, Table 12-9 | "Programmable vision coprocessors natively supported by the FTC SDK are an exception to this rule, and reprogramming them is permitted." Table 12-9 lists "Limelight Vision, Limelight 3A, LL_3A". Checklist: "Teams are not allowed to modify software on coprocessors (except for LimeLight 3A)." | The Limelight 3A is the one allowed programmable vision coprocessor. The FTC SDK 12.0 in this repo contains `Limelight3A`, so it is natively supported. Custom pipelines and Python SnapScripts on the Limelight are "reprogramming", which R702 permits for it. | **Verified.** Manual R702 and Table 12-9; checklist Control System item 2. SDK class: checked in the repository. |
| Other Limelights | R702, Example 6 | "The OpenMV Cam, Luxonis OAK-1, and LimeLight Vision Limelight 3G are examples of programmable vision coprocessors that are prohibited." | Use only the 3A. | **Verified.** The Limelight 4 is not mentioned by name. It is absent from Table 12-9, so by that table it is not on the allowed list. That conclusion is the auditor's reading. See open question 1. Manual R702. |
| Limelight is on USB legally | R707, R708 | R707: "Only the following devices may be connected to the ROBOT control system using USB: A. webcams and optical vision sensors per R708, B. USB hub or USB switch, and C. a REV Expansion Hub." R708: "Only single image sensor vision devices that are natively supported by the ROBOT CONTROLLER app are allowed to connect to USB (stereoscopic cameras are not allowed). This includes ... B. Vision coprocessors allowed per R702." | The Limelight 3A has one image sensor (OV5647, docs) and is a vision coprocessor allowed by R702. It connects by USB to the Control Hub. | **Verified.** Manual R707, R708; checklist item 3. Sensor: Limelight 3A docs (link above). |
| Number of vision devices | R707, R708 | Neither rule states a limit on the number of cameras or coprocessors. | Beacon uses one Limelight. | **Verified** (by absence of any numeric limit in R707, R708, R701). The checklist also states none. |
| What the Limelight may compute | R702 | The rule bars altering "coprocessor software" except for natively supported programmable vision coprocessors, where "reprogramming them is permitted". No rule limits what that coprocessor computes. | AprilTag detection and MegaTag2 pose fusion run on the Limelight. R202 C still bars tag-like imagery on the robot. | **Verified** (no restriction found; the manual was searched for "Limelight", "vision", "camera"). |
| Limelight power | R602, R613 A, R611 | R602: "Any device receiving signals from a REV Control or Expansion Hub must be powered by the main ROBOT battery." R613 A: "sensors, encoders, and other devices must be powered solely by the power regulation device they are connected to. Do not use outside power sources". R611: powered USB hubs only from "an approved COTS USB battery Pack per R602" or the hub 5V auxiliary port. | The Limelight takes its power (4.1 to 5.75 V, up to 4 W, docs) only from the Control Hub USB 3.0 port. Do not power it from a USB battery pack. If a USB hub is ever used, power it per R611. | **Verified** for the rules. The 4 W figure is from the Limelight docs. Whether the Control Hub's USB 3.0 port can supply the Limelight's full draw is **Unverified** (REV's port page states no current figure). Test it. |
| Limelight configuration and R704 | R704 C, D; E301 | C: "Ensure all programming laptops and other devices (other than the DRIVER STATION device) are disconnected from the ROBOT CONTROLLER Wi-Fi network during MATCH play." D: "Software may only stream robot control data, debugging data, and telemetry to and from the ROBOT using the FTC Driver Station Application. Additional logging/streaming services ... are prohibited. No continuous video stream is allowed." | The Limelight is configured in a browser at `http://limelight.local:5801` with the laptop connected to the Limelight by USB (docs). That path does not use the RC Wi-Fi. In a match nothing is connected and nothing is streamed. Do not use FTC Dashboard, Panels or similar. | **Verified** for the rules. Whether the web interface or camera stream can be reached over the Control Hub's Wi-Fi is **Unverified** (docs say only that the Limelight appears as an "Ethernet Device" on the hub). See open question 3. |
| Limelight wireless | R704 A; E301 | R704 A: "Teams may not use any other form of wireless communication, except those offered by the provided official tools, to communicate to, from, or within the ROBOT." | The Limelight 3A docs list USB-C connectivity only ("USB-C connectivity only (no RJ45 port)"). The docs name no Wi-Fi or Bluetooth radio. | **Unverified** that the 3A has no active radio. The docs list none; the comparison table lists only USB-C and Ethernet-class I/O. Ask Limelight or the Q&A if an inspector raises it. |
| Pinpoint allowed | R303 J, R702 examples | R303 J allows "dead-wheel odometry kits". R702 Example 2: a "laser and IMU tracking device that uses an onboard microcontroller ... Firmware updates provided by [the maker] are allowed to be applied to the device. This device is allowed." Example 3: updates "provided in binary form by the manufacturer ... may be applied". | The goBILDA Pinpoint is a COTS odometry computer with an onboard processor and IMU, used with two COTS odometry pods. Keep manufacturer firmware. No rule names the Pinpoint; the SDK has a built-in driver (`GoBildaPinpointDriver`, checked in the repository). | **Verified** for the rules. **Unverified** that any rule names the Pinpoint: the manual contains no mention of it (searched for "Pinpoint"). It is allowed by the examples above and by the absence of a bar. See open question 2. |
| Sensors and I2C | No rule | No rule in Section 12 restricts I2C sensors by type or number. R609 sets 28 AWG minimum for "signal level circuits" such as I2C but exempts "wires originally attached to legal COTS devices or wires included/sold by the manufacturer". | Use the cables sold with the Pinpoint and pods. Do not extend or splice them. | **Verified** (by absence of a sensor rule). Manual R609. |
| Pinpoint power | R613 A, R607 | R613 A as above. R607: "Any active electrical item that is not an actuator ... or power regulation device ... is considered a CUSTOM CIRCUIT. CUSTOM CIRCUITS shall not provide regulated output voltages exceeding 5V". | The Pinpoint takes 3.3 to 5 V at 80 mA (docs) from the Control Hub I2C port and produces no output above 5 V. The pods are powered by the Pinpoint. Power nothing else from outside the hub. | **Verified** for the rules. Pinpoint ratings from the user guide. |
| Pinpoint lighting | R202 F, J | F: "high intensity light sources ... should only be illuminated for a brief time while targeting and may invite additional scrutiny". J: "decorative or functional lighting which flashes at greater than 5Hz will invite additional scrutiny". | The Limelight 3A has "No built-in LED illumination" and one green status light (docs). The Pinpoint has a small RGB status LED (green when ready). Do not add a ring light or any lamp. | **Verified** for the rules. **Corrected** from pass 1 only in that the spec called R202 F and J hard bans; the manual says "additional scrutiny". Docs for the Limelight and Pinpoint LEDs. Whether the Limelight status light blinks faster than 5 Hz is **Unverified**; it is a small indicator, not a lamp. |
| No AprilTag imagery on robot | R202 C | "imagery on your ROBOT that utilizes or closely mimics 36h11 AprilTags". | Nothing tag-like on the robot, including the Limelight mount and any calibration target. | **Verified.** Manual R202. |
| Radios on the Control Hub | R711 A, C | A: "REV Control Hub users must change the Wi-Fi password to a non-default password". C: "on ROBOT CONTROLLER and DRIVER STATION Android devices, Wi-Fi must be enabled and Bluetooth must be disabled". | Change the password. Wi-Fi stays on (the Driver Station talks to the Control Hub over it). Bluetooth stays off. Airplane Mode is not required for a Control Hub (B applies to "smartphone users"). | **Corrected.** Pass 1 required Airplane Mode (rule B) for the phone. Not applicable to a Control Hub. The checklist ties the self-inspect screens (no red X) to R711. Manual R711; checklist Control System item 5. |
| RC name | R705 | "ROBOT CONTROLLER should be named <team number>-RC (e.g., 12345-RC)". A spare may be "<team number>-<letter>-RC". | Name the Control Hub `<team>-RC` as the ROBOT CONTROLLER Instructions describe. | **Verified.** Unchanged from pass 1. Manual R705. |
| Do not modify the controller | R706 | The "Android-based ROBOT CONTROLLER device ... shall not be tampered with, modified, or adjusted" (drilling, cutting, painting, replacing enclosures, etc.). Exceptions include B: "fasteners (including adhesives) may be used to attach devices ... to the ROBOT or to secure cables to the device", C: thermal interface material, F: "device firmware may be updated with manufacturer supplied firmware". | Mount the Control Hub with its mounting holes or fasteners. Do not open or paint it. | **Verified.** Manual R706. The ambiguity about "software" in R706 (pass 1 open question 2) is much smaller now: the only code on the Control Hub is the team's normal TeamCode app. |
| Inspectable lights | R606 B | "the ROBOT CONTROLLER device must be mounted on the ROBOT such that the diagnostic lights, or device screen if applicable, can be visible for inspection." Not required "in STARTING CONFIGURATION or normally during a MATCH". | Mount the Control Hub so its status LEDs can be shown to the inspector. Do the same for the Pinpoint LED, to help field staff. | **Corrected.** Pass 1 was about the phone screen. For a Control Hub it is the diagnostic lights. Manual R606. |
| Hub power | R603, R608 | R603: "Exactly one main power switch must control all power provided by the ROBOT battery pack to all power regulating devices on the ROBOT". R608: the REV Control Hub and Expansion Hub are "Only powered using the XT30 connectors on the device by the ROBOT main battery". | Battery to main switch to the Control Hub XT30. The Expansion Hub, if used, gets power through an XT30 extension cable from the Control Hub. | **Verified.** Unchanged from pass 1 (both rules cover both hubs). Manual R603, R608. |
| Phone power exemption | R602 B-ii | COTS USB battery packs may power "ROBOT CONTROLLER smartphones". | Not used. The same rule means a USB battery pack must not power the Limelight (see Limelight power). | **Verified.** Manual R602. |
| Actuator count | R503 | "A ROBOT may not have more than 8 motors and 8 servos ... for all MECHANISMS used in all configurations." | A Control Hub has 4 motor ports and 6 servo ports; with one Expansion Hub: 8 motor ports, 12 servo ports. Stay at 8 motors and 8 servos. The odometry pods are encoders, not motors. | **Verified.** Manual R503. Port counts: Control Hub ports page (link above). |
| Odometry pods | R303 J | Dead-wheel odometry kits are an exception to the single degree-of-freedom limit. Example 2: dead wheel modules are "typically composed of a 1 DoF wheel ... and a spring force ... creating a 2 DoF system" and are allowed. | The two goBILDA pods are a dead-wheel kit. | **Verified.** Manual R303. |
| Software and outside help | R304, R101 | R304: "ROBOT software, designs, and FABRICATED ITEMS created before Kickoff are permitted." R101's intent: "to discourage complete solutions which are provided wholly by outside organizations or companies." | Beacon is open source. The team must be able to explain its localizer. | **Verified.** Unchanged. Manual R304, R101. |
| Re-inspection | I301 | "Teams should demonstrate to INSPECTORS all ways the ROBOT will be configured for a MATCH." | Bring the robot as it will run. | **Verified.** Manual I301, 3.3.2. |
| Starting position | G304 | A ROBOT must be "touching the FIELD perimeter wall", "confined to its STARTING CONFIGURATION", and "fully motionless following completion of OpMode initialization". | The Pinpoint calibrates its gyro zero offset with the robot stationary. It does this when first powered (about 0.25 s, user guide), and again when `resetPosAndIMU()` or `recalibrateIMU()` is called. The guide recommends `resetPosAndIMU()` "when you start the first OpMode of your match". Plan: robot placed against the wall, INIT pressed, robot untouched until the Pinpoint LED is green. This agrees with G304 H. | **Corrected.** Pass 1 treated G304 H as a risk for a VIO that needs motion to start. For this build it is the opposite: the robot must stay still, and the rule allows that. Manual section 11, G304; Pinpoint user guide, "Zeroing/Calibrating the IMU". |

## Not applicable to this build (phone Robot Controller tiers are parked)

The phone tiers (ARCore, VINS) are parked in the repository and may return. Rows from pass 1
that apply only to a phone Robot Controller are kept short here. If a phone comes back, redo the
audit from the first-pass version in git history and re-check against the manual then current.

- **R701 B / R703 phone connection:** a phone connects to an Expansion Hub by "any combination of
  USB cables ... and/or USB hubs". Not used.
- **R711 B, C for phones:** "smartphone users must enable Airplane Mode". Not used.
- **Which phones, Android and RC app version:** the manual set no approved list or Android version.
  Not used.
- **R602 B-ii:** a USB battery pack may power the RC smartphone. Not used.
- **R606 B phone screen:** replaced by the Control Hub diagnostic lights row above.
- **R706 phone modification, case, fan, heat:** the phone-specific questions (cases, fans, heat
  spreaders) are dropped. Thermal and fan questions do not arise for the Control Hub.
- **Phone camera and IMU for navigation, Google Play Services for AR, NFC and UWB, custom app with
  ARCore and native library:** not used.
- **UVC webcam fallback behind a USB hub (R707 A, R708 A, R703, R707 B):** not used. A webcam would
  also need the Control Hub USB 3.0 port that the Limelight already uses.
- **One-phone-only (I302):** not used. I302 still counts Android devices: the Control Hub is one,
  and the Driver Station device is the other.

## Open questions for the official Q&A

The Q&A is at https://ftc.game/qa (redirects to https://game-qa.firstinspires.org/boards/2027/FTC/QA).
Search for existing answers first. The manual does not clearly answer these.

1. **Which Limelight.** "Table 12-9 lists the Limelight 3A (LL_3A). Is the Limelight 4 permitted?"
   Only if the team owns or wants one. The 3A is the safe choice.
2. **goBILDA Pinpoint.** "Is the goBILDA Pinpoint Odometry Computer (3110-0002-0001), which contains
   an IMU and a processor, permitted as an I2C sensor under R702 and R303 J when used with its
   manufacturer firmware?" The rules and examples point to yes. A written answer helps with an
   inspector who has not seen one.
3. **Limelight web interface.** "May a team configure a Limelight 3A in a browser, with a laptop
   connected to the Limelight by USB, in the pits? If the laptop reaches the Limelight interface
   through the Control Hub's Wi-Fi, does R704 D (no additional streaming, no continuous video
   stream) apply?" The team plan is USB-direct only, with the Limelight out of the Control Hub,
   so ask only if you want to configure over the robot network.
4. **Limelight status light and R202.** Only if an inspector raises it: the 3A has no illuminator,
   and one green status light.
5. **Control Hub USB 3.0 power budget.** Not a rules question. Test with the Limelight running at
   full frame rate and watch for brownouts and Wi-Fi disconnects. REV warns that the Control Hub's
   USB 2.0 port can cause Wi-Fi disconnects through electrostatic discharge, and tells teams to
   plug cameras into the USB 3.0 port (REV/FTC docs, Control Hub ports page).
6. **Alignment jig.** No rule mentions placement jigs. "May a team use a handheld template to
   position the ROBOT against the perimeter wall during pre-MATCH setup, if it is removed before
   the MATCH?" (See G304 and the pre-MATCH setup text in section 10.)
7. **Limelight frames to the Control Hub (added 2026-11-17, ask this one).** "Beacon reads the
   Limelight 3A's camera frames over its USB connection into the team's Robot Controller app on
   the Control Hub, during the MATCH, for on-robot localization. Nothing leaves the ROBOT. Is this
   permitted, and is it outside R704 D's 'No continuous video stream is allowed'?" Unaudited:
   R704 D reads as covering streams to or from the ROBOT, but the manual does not say so
   explicitly.

## Inspection checklist (tick each item)

Bring this file, `WIRING.md`, the user guide pages for the Pinpoint and Limelight 3A, and the
manual rule text printed or on a device that is not the robot's Control Hub. Rule numbers are
from the TU09 manual.

Controller and USB
- [ ] A REV Control Hub (REV-31-1595) is the ROBOT CONTROLLER (R701).
- [ ] No more than one Expansion Hub (R701), connected by the RS485 cable.
- [ ] The Limelight is a Limelight 3A, LL_3A (R702, Table 12-9). Not a 3G, not a 4.
- [ ] Only the Limelight (and an Expansion Hub or USB hub if used) is on USB (R707, R708).
- [ ] The Limelight runs only the Limelight 3A's own firmware, with the pipelines and Python you wrote on it (R702).
- [ ] The Pinpoint runs manufacturer firmware only (R702 examples, R706 F).
- [ ] Any powered USB hub is powered from a hub 5V auxiliary port or a legal USB battery pack (R611). No hub is the normal case.

Control Hub configuration (show the RC self-inspect screen, no red X icons: R711)
- [ ] Wi-Fi password changed from the default (R711 A).
- [ ] Wi-Fi on, Bluetooth off (R711 C).
- [ ] Control Hub named `<team number>-RC` (R705).
- [ ] Control Hub unmodified: not opened, drilled or painted (R706).
- [ ] Control Hub status lights can be shown to the inspector (R606 B).

Software and wireless
- [ ] No laptop or other device on the RC Wi-Fi during a match (R704 C).
- [ ] No FTC Dashboard, Panels or other streaming or logging service, and no video stream (R704 D).
- [ ] Limelight configuration done by laptop plugged directly into the Limelight (R704, see question 3).
- [ ] No AprilTag-like imagery anywhere on the robot (R202 C).
- [ ] No ring light or lamp on the Limelight (R202 F, J).
- [ ] The team can explain, in its own words, what the localizer does (R101 intent).

Power and actuators
- [ ] One main battery and one main power switch controlling all hubs (R601, R603).
- [ ] Control Hub (and Expansion Hub) powered only through their XT30 connectors from the main battery (R608).
- [ ] The Limelight is powered from the Control Hub USB port only. The Pinpoint from the I2C port only (R602, R613 A).
- [ ] 8 or fewer motors and 8 or fewer servos in all configurations (R503).
- [ ] Wire sizes and colors per R609 and R610, or manufacturer cables unmodified.

Setup and event
- [ ] Robot shown in every configuration it will use (I301).
- [ ] Re-inspection requested after any change (3.3.2).
- [ ] Match start: robot placed against the wall, INIT pressed, robot left alone until the Pinpoint LED is green (G304 H).
- [ ] The auto start jig, if used, is a handheld team tool and leaves the field before the match (question 6).
- [ ] Re-read `tu-latest` on the Game and Season Materials page the week of the event.

## Not checked in this audit

- The official Q&A archive (could not be searched).
- Anything on real hardware, including the Limelight's current draw, the Limelight status light
  behavior, and whether the Limelight interface is reachable over the Control Hub Wi-Fi.
- The goBILDA Pinpoint's firmware policy beyond the user guide.

The Head REFEREE and the Lead Robot Inspector decide at the event.
