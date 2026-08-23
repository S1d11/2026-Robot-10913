# FRC Robot Code — Team 10913 (2026)

Command-based Java robot code for a swerve-drive robot with shooter, hopper, intake, AprilTag vision, PathPlanner autos, Elastic telemetry, and desktop simulation.

## Robot systems

- **Drive:** four MAXSwerve modules, SparkFlex drive motors, SparkMax steering motors, and a Pigeon 2 gyro.
- **Shooter:** two SparkFlex NEO Vortex motors (CAN 13 leader, CAN 14 follower) with closed-loop flywheel speed control.
- **Hopper:** one SparkMax feeder motor (CAN 12).
- **Intake:** roller motor (CAN 11) and lift motor (CAN 10).
- **Vision:** three PhotonVision cameras that fuse valid AprilTag estimates into drivetrain pose estimation.

## Controller mapping

### Driver — port 0

- Left stick: field-relative translation
- Right-stick X: rotation when hub tracking is disabled
- Right stick press: X-stance
- Right bumper: enable hub tracking
- Left bumper: disable hub tracking
- Start: zero gyro heading
- Back (disabled, wheels straight forward): save swerve module offsets

### Operator — port 1

- A: launch at the selected RPM once the shooter reaches speed
- B: eject intake, hopper, and shooter
- Right trigger: run intake roller
- Left bumper: outtake roller
- X / Y: deploy / retract intake lift
- Right bumper: spin shooter only
- D-pad left / up / right: close / medium / distance shooter presets
- Back (disabled, intake physically retracted): zero the intake lift encoder

## Autonomous

PathPlanner assets live in `src/main/deploy/pathplanner`. The Elastic chooser exposes only the vetted competition autos; edit `COMPETITION_AUTO_NAMES` in `RobotContainer` when approving a new routine. The PathPlanner named commands are:

- `DeployIntake`, `RetractIntake`, `StartIntake`, `StopIntake`
- `SpinUpShooter`, `Shoot`, `StopShooter`

Timed autonomous shooting uses PathPlanner deadline groups so a shot is stopped cleanly when its wait duration ends.

Every event marker must contain a named-command payload. A marker name alone is only an `EventTrigger`; this project intentionally uses direct named commands in the path files.

## Pre-match calibration

Before enabling the robot:

1. With the robot disabled, point all four swerve wheels straight forward and press **driver Back** once. The offsets are saved persistently on the roboRIO.
2. With the robot disabled, physically retract the intake lift and press **operator Back** once. Automatic lift motion is blocked until this is done on real hardware.
3. Confirm `Drive/Module Offsets Calibrated` and `Intake/Lift Calibrated` in Elastic.

The 2026 hub status on the dashboard follows Driver Station Game Data and match-time shifts. It is informational; shooter controls remain under operator control.

The current PathPlanner mass and moment of inertia are derived from the drivetrain's 54 kg, rectangular bumper model. Re-measure those values after significant robot changes and update both the constants and PathPlanner settings together.

## Build and deploy

Use the WPILib 2026 toolchain with Java 17.

```bash
# macOS / Linux
chmod +x gradlew
./gradlew build
./gradlew deploy
./gradlew simulateJava

# Windows
gradlew.bat build
```

The team number is configured in `.wpilib/wpilib_preferences.json`.

## Tuning locations

- `Constants.java`: drivetrain geometry, CAN IDs, operator ports, and field targets
- `ShooterConstants.java`: flywheel gains, presets, and distance-to-RPM table
- `HopperConstants.java`: feeder gains and speed
- `IntakeConstants.java`: intake/lift voltages and lift positions
- `VisionConstants.java`: camera names, transforms, and validation thresholds
- `src/main/deploy/pathplanner/settings.json`: PathPlanner physical model; keep its module locations, gearing, motor type, speed limit, mass, and MOI aligned with the real robot

Validate all mechanism limits, camera transforms, and shot presets on the real robot before competition use.
