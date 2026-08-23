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

### Operator — port 1

- A: launch at the selected RPM once the shooter reaches speed
- B: eject intake, hopper, and shooter
- Right trigger: run intake roller
- Left bumper: outtake roller
- X / Y: deploy / retract intake lift
- Right bumper: spin shooter only
- D-pad left / up / right: close / medium / distance shooter presets

## Autonomous

PathPlanner assets live in `src/main/deploy/pathplanner`. Select an auto through the Elastic chooser. The PathPlanner named commands are:

- `DeployIntake`, `RetractIntake`, `StartIntake`, `StopIntake`
- `SpinUpShooter`, `Shoot`, `StopShooter`

Timed autonomous shooting uses PathPlanner deadline groups so a shot is stopped cleanly when its wait duration ends.

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

Validate all mechanism limits, camera transforms, and shot presets on the real robot before competition use.
