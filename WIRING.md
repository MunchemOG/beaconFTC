# Beacon wiring, mounting and start-placement notes (Control Hub build)

Print this page and keep it with `RULES.md` on the robot cart. Rule numbers refer to the
2026-27 BIOBUZZ Competition Manual, TU09 (https://ftc-resources.firstinspires.org/ftc/game/cm-html).
Audit pass 2: 2026-11-17. The Head REFEREE and Lead Robot Inspector decide at the event.

Hardware in this build: REV Control Hub, optional REV Expansion Hub, goBILDA Pinpoint Odometry
Computer (3110-0002-0001) with two odometry pods, Limelight 3A. There is no phone on the robot.

Marks: **Verified** = read in the manual or in manufacturer/FTC documentation this session.
**Corrected** = differs from the spec, from the first version of this file, or from the notes
another session appended to it. **Recommendation** = team practice, not a rule or a documented
fact. **Unverified** = could not be confirmed. Nothing here was tested on hardware.

## Diagram

```
   12 V battery --> main power switch --> XT30 ---------------------+
                                                                    |
                                                                    v
   +----------------------------------------------------------------------+
   |  REV CONTROL HUB (REV-31-1595)      name <team>-RC                  |
   |                                                                      |
   |  XT30 (power in)  <- main switch                                     |
   |  XT30 (second)    -> XT30 extension cable -> Expansion Hub (option)  |
   |  RS485 (3-pin JST-PH) -> Expansion Hub (option)                      |
   |  I2C port 1, 2 or 3 (4-pin JST-PH) <- Pinpoint        (not port 0)   |
   |  USB 3.0 (blue, Type-A) <- USB-C-to-A cable <- Limelight 3A          |
   |  motor ports 0-3, servo ports 0-5, encoders, digital, analog         |
   +----------------+---------------------------+-------------------------+
                    |                           |
          (optional)| RS485 + XT30 ext.         | 4-pin JST-PH (I2C)
                    v                           v
   +-------------------------------+   +----------------------------------+
   | REV EXPANSION HUB (REV-31-1153)|  | goBILDA PINPOINT (3110-0002-0001)|
   | address 2 (default), no change |  | I2C address 0x31                 |
   | motor 0-3, servo 0-5, ...      |  | X pod port <- forward pod        |
   +-------------------------------+   | Y pod port <- sideways pod       |
                                       | IMU inside; status LED           |
                                       +-----------+----------------+-----+
                                                   |                |
                                          X pod (tracks forward)   Y pod (tracks sideways)
                                          2000 CPR, 32 mm wheel    (goBILDA 4-Bar Odometry Pod)

   Control Hub USB 3.0 -- USB-C cable --> LIMELIGHT 3A  (powered from the hub USB port only)

   Totals: Control Hub 4 motor + 6 servo ports; with one Expansion Hub 8 motor + 12 servo ports.
   Rule limit: 8 motors and 8 servos in total (R503).
```

A laptop for Limelight set-up plugs directly into the Limelight, never into the Control Hub's
Wi-Fi, and is unplugged before the match (R704 C).

## What each connection is, and the source

| Connection | Fact | Mark and source |
| --- | --- | --- |
| Controller | A REV Control Hub (REV-31-1595) is the ROBOT CONTROLLER; at most one added REV Expansion Hub. | **Verified**, R701. |
| Hub power | The Control Hub and Expansion Hub are "Only powered using the XT30 connectors on the device by the ROBOT main battery". Battery goes through the main power switch. | **Verified**, R603, R608. |
| Hub to hub power | "An XT Extension Cable to provide power between the Control Hub and the Expansion Hub." The yellow XT30 connectors "are used to power your REV Hub as well as all the devices connected to it." | **Verified**, REV: https://docs.revrobotics.com/duo-control/adding-more-motors/adding-an-expansion-hub ; FTC docs ports page: https://ftc-docs.firstinspires.org/control_hard_compon/rc_components/hub/ports/ch-ports.html |
| Hub to hub signal | "Use a 3-pin JST PH cable to connect the RS485 port on the Control Hub to the Expansion Hub." The RS485 ports are 3-pin JST-PH; "Both RS485 ports can be used to add redundancy". | **Verified**, REV page above; FTC ports page. |
| I2C ports | 4-pin JST-PH style, "Each port is a single I2C bus". | **Verified**, FTC ports page. |
| I2C port 0 | A Pedro Pathing document says to avoid port 0 for the Pinpoint because the Control Hub's built-in IMU is on it. Use port 1, 2 or 3. | **Unverified** against REV; seen only as a search snippet of a secondary source. Safe either way. |
| Pinpoint to hub | The Pinpoint's connector type is "4-Pos JST PH [FH-MC]", I2C address 0x31, tested up to 400 kHz. Input 3.3 V to 5 V, 80 mA. Wire it to a hub I2C port with the cable goBILDA supplies. In the Robot Controller configuration the device type is "goBILDA Pinpoint Odometry Computer". | **Verified**, Pinpoint user guide: https://servocity.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf . Which cable ships in the kit and its wire order: **Unverified**; do not make your own. |
| Pods to Pinpoint | One pod is the X pod (tracks forward) and one the Y pod (tracks sideways). Mount the Pinpoint "sticker-side up on the chassis". The X pod counts up when the robot moves forward; the Y pod counts up when it moves left. Reverse an encoder direction in software if either is backwards. | **Verified**, Pinpoint user guide, setup steps 1, 2, 7, 8 and "Pinpoint Required Configuration". |
| Pinpoint status LED | Green = Ready. Red = Calibrating or Not Ready (power-up). Purple = no pods. Blue = X pod not detected. Orange = Y pod not detected. | **Verified**, Pinpoint user guide, "Device State Codes". |
| Limelight to hub | "Run a USBC to USBA cable from your Limelight 3A to your Control Hub's USB 3.0 Port" (the blue port). In Configure Robot, scan: an "Ethernet Device" appears; rename it `limelight`. Power input 4.1 V to 5.75 V over USB, 4 W maximum. | **Verified**, Limelight 3A quick-start: https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a . Whether the hub's USB 3.0 port can supply the full 4 W: **Unverified**. |
| Why not USB 2.0 | "An electrostatic discharge (ESD) event on the USB 2.0 port on the Control Hub can cause Wi-Fi disconnects ... Ensure that you plug USB devices, such as a Camera, into the USB 3.0 Port". | **Verified**, FTC ports page (link above). |
| Limelight web interface | After connecting the Limelight to a computer by USB, wait for the green light (15 to 20 s), then open the Limelight Hardware Manager or `http://limelight.local:5801`. Set the team number and click "Restart Vision Client". Changes save automatically. | **Verified**, Limelight quick-start. Rule bearing: R704 C and D; see `RULES.md`. |
| Wire rules | Signal-level circuits (I2C, RS485, encoders) need 28 AWG or larger, but manufacturer-supplied wires are exempt. 12 V battery wiring needs 18 AWG or larger. 12 V and +5 V aux wires must be color-coded along their whole length. | **Verified**, R609, R610. |

### Changes from the earlier version of this file

- **Corrected:** the "Odometry tier wiring" notes appended by another session said Pinpoint offsets
  are "measured from the centre of rotation". The Pinpoint guide says offsets are measured from a
  tracking point, which may be any point on the robot. For a two-dead-wheel setup the tracking
  point "does not affect the accuracy of the system". The offsets describe where each pod sits
  relative to that point: the X pod offset is how far sideways the X pod is (left positive); the Y
  pod offset is how far forward the Y pod is (forward positive). Both are in millimetres in the
  Pinpoint driver.
- **Corrected:** the same notes said to keep the robot still "for about a second" for the IMU.
  The guide gives about 0.25 s for the calibration. Keep the robot still until the LED is green.
  "About a second" is a safe habit, but it is not the documented figure.
- **Kept, Verified:** Pinpoint on an I2C port as device type "goBILDA Pinpoint Odometry
  Computer"; X pod up going forward, Y pod up going left; Limelight in the Control Hub USB 3.0
  port, named `limelight`.
- **Unverified:** "camera pose on the robot set in the Limelight web UI" and "AprilTag pipeline at
  index 0". The Limelight docs page read for this audit did not show where the robot-space camera
  offset is set. Check it in the web interface.
- **Corrected (phone build):** the earlier "Changing the second hub's address" procedure was for a
  phone Robot Controller, where two new Expansion Hubs both default to address 2. REV's page says
  addresses "may need to change" only when "an Android device as the Robot Controller" is used.
  With a Control Hub plus one Expansion Hub, no change is needed: the Expansion Hub keeps its
  default address 2 and the Control Hub is separate. That the Control Hub uses address 1 comes
  from a secondary source (FTC wiki) and is **Unverified**. If you ever need a change, it is made
  in the Robot Controller app under Advanced Settings, Expansion Hub Address Change, not in the
  REV Hardware Client.

## Strain relief (Recommendation)

No rule requires these. They stop the loose-cable failures that cost matches.

- **Limelight USB-C.** The USB-C connector is small. Anchor the cable with a zip tie or clamp 2 to
  3 cm behind the connector, fixed to the mount, with a small service loop. Use a snug cable;
  a loose USB-C plug will drop out and take the absolute fixes with it. R706 B allows fasteners
  "to secure cables to the device".
- **Pinpoint I2C cable.** If the I2C cable pulls out mid-match, the pose stops. Secure it near the
  Pinpoint and near the hub, and keep it clear of the pods' moving parts and of drive wheels.
- **Pod cables.** Route away from the wheels. Leave enough slack for the pod's spring travel.
- **RS485 and XT30 extension** (if an Expansion Hub is used): secure so neither can be pulled
  when the robot is hit. Do not extend or splice them (R609 exempts manufacturer wires only).
- After assembly, wiggle every connector by hand with the robot powered and watch the Driver
  Station and the Pinpoint LED. Repeat after every collision in practice.
- Do not modify the hubs, Pinpoint, or Limelight to add connectors (R706; R702 firmware rules).

## Mounting (Recommendation, with rule limits)

- **Pinpoint.** Mount it sticker-side up and rigid on the chassis. It contains the IMU. Keep it
  away from large motors if you see heading noise (a team observation, not documented).
- **Pods.** One tracks forward (X) and one sideways (Y). Each must keep wheel contact with the tile.
  Dead-wheel kits are legal (R303 J). Measure each pod's offset from your chosen tracking point.
- **Limelight 3A.** Rigid, with a clear view of the field walls' AprilTags. It is 72.1 mm by
  48.1 mm by 16.8 mm, 0.20 lb, with 4x #10 through holes, 4x M3 and 6x M4 threaded holes
  (Limelight quick-start). Field of view 54.5 degrees by 42 degrees. Keep mechanisms out of view.
  Write the camera's position on the robot in the Limelight web interface and in `BeaconHardwareConfig`
  to the same numbers, in the same frame and units; measure with a tape, not by eye.
- **Control Hub.** Fasten it so its diagnostic lights can be shown to the inspector (R606 B). Do
  not open, drill or paint it (R706). Leave the Wi-Fi antenna area clear of metal.
- **No tag imagery.** Nothing on the robot may mimic AprilTags (R202 C). No ring light on the
  Limelight (R202 F, J); the 3A has no built-in LED illumination (Limelight docs).

## Auto-start placement jig

`docs/archive/astro-spec/specs/05-CALIBRATION.md` section C asks for this note. The Pinpoint reports
position relative to where it was zeroed. At the start of autonomous the localizer needs the robot
at a known place in the field frame, and a jig gets the same place every time.

- **What it is.** A rigid corner template, handheld, printed or cut from stiff board or plastic.
  It registers against the field perimeter wall and one edge of a tile, and the robot's frame or
  bumper registers against the template.
- **Why the wall.** The rules require the robot to be set up "touching the FIELD perimeter wall"
  and "confined to its STARTING CONFIGURATION" (G304 C, F), so one side of the robot is already
  fixed by the wall.
- **Build it.**
  1. Measure the robot's starting configuration against the wall, with the robot square to it.
  2. Make a right-angle template with one leg along the wall and the other leg along the tile
     line you choose as the starting mark. Make the legs long enough to register by touch.
  3. Mark the template with the robot's footprint so the person placing the robot can see it.
  4. Write the starting pose that results into the config and the Autonomous OpMode's
     `setPose` call, in metres and radians, in the field frame. Measure it with a tape and check
     it against a Limelight AprilTag fix on the practice field.
  5. Make one jig per starting position (red and blue, near and far), and label each.
- **Order of events at the field (G304 H and the Pinpoint IMU).**
  1. Place the jig and the robot against it, then remove the jig.
  2. Press INIT. The Pinpoint's gyro zero offset must be measured with the robot stationary; the
     guide says a bad offset makes heading drift while the robot is not moving. Call
     `resetPosAndIMU()` (or `recalibrateIMU()`) in init and wait for the green LED.
  3. Do not touch or bump the robot until the match starts. G304 H requires the robot to be
     "fully motionless following completion of OpMode initialization".
  4. If the robot is moved after INIT, re-run the calibration with it still before the match.
- **Rule status.** No rule found mentions jigs. Whether a handheld template may be used at the
  field during pre-match setup is an open question (`RULES.md`, question 6). Until it is answered,
  place the robot by eye, using the field wall and tile lines, whenever a referee objects.

## Not verified

- Anything on real hardware. No hub, Pinpoint, pod or Limelight was connected during this audit.
- The Control Hub USB 3.0 port current limit versus the Limelight's 4 W maximum.
- The cable that ships in the goBILDA Pinpoint kit and its connector pinout.
- Whether the Limelight status light blinks, and at what rate.
- That I2C port 0 on the Control Hub is shared with its built-in IMU (secondary source only).
- Menu names in a newer Robot Controller app or Limelight OS than those the docs describe.
