package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.ElasticTelemetry;

public class Intake extends SubsystemBase {

  private final SparkMax motor;

  private final SparkMax liftMotor;

  private final RelativeEncoder encoder;

  // lift motor and encoder

  private final RelativeEncoder liftEncoder;

  private boolean liftCalibrated = RobotBase.isSimulation();
  private boolean hasReportedUncalibratedLift = false;

  public Intake() {

    motor = new SparkMax(intakeMotorCanId, MotorType.kBrushless);

    liftMotor = new SparkMax(intakeLiftMotorCanId, MotorType.kBrushless);

    encoder = motor.getEncoder();

    liftEncoder = liftMotor.getEncoder();

    var config = new SparkMaxConfig();

    config
        .inverted(intakeMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(intakeCurrentLimit)
        .voltageCompensation(12.0);

    config.encoder.velocityConversionFactor(1.0);

    var liftConfig = new SparkMaxConfig();

    liftConfig
        .inverted(intakeLiftMotorInverted)
        .smartCurrentLimit(intakeCurrentLimit)
        .voltageCompensation(12.0);

    motor.configure(
        config,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    liftMotor.configure(
        liftConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    if (liftCalibrated) {
      liftEncoder.setPosition(retractedPosition);
    }
  }

  @Override
  public void periodic() {
    ElasticTelemetry.setNumber("Intake/Roller RPM", encoder.getVelocity());
    ElasticTelemetry.setNumber("Intake/Lift Position", liftEncoder.getPosition());
    ElasticTelemetry.setBoolean("Intake/Is Deployed", isLiftDeployed());
    ElasticTelemetry.setBoolean("Intake/Is Retracted", isLiftRetracted());
    ElasticTelemetry.setBoolean("Intake/Lift Calibrated", liftCalibrated);
  }

  public void intake() {

    motor.setVoltage(intakeVoltage);
  }

  public void liftRetract() {
    if (!canMoveLift()) {
      return;
    }

    if (getLiftPosition() > retractedPosition + liftPositionTolerance) {

      liftMotor.setVoltage(-liftVoltage);

    } else {

      liftStop();
    }
  }

  public void liftDeploy() {
    if (!canMoveLift()) {
      return;
    }

    if (getLiftPosition() < deployedPosition - liftPositionTolerance) {

      liftMotor.setVoltage(liftVoltage);

    } else {

      liftStop();
    }
  }

  public void outtake() {

    motor.setVoltage(outtakeVoltage);
  }

  public void feed() {

    motor.setVoltage(feedVoltage);
  }

  public void stop() {
    motor.stopMotor();
    liftStop();
  }

  public void liftStop() {

    liftMotor.stopMotor();
  }

  public double getVelocityRPM() {

    return encoder.getVelocity();
  }

  public double getLiftPosition() {

    return liftEncoder.getPosition();
  }

  public boolean isLiftDeployed() {

    return liftCalibrated && Math.abs(getLiftPosition() - deployedPosition) < liftPositionTolerance;
  }

  public boolean isLiftRetracted() {

    return liftCalibrated
        && Math.abs(getLiftPosition() - retractedPosition) < liftPositionTolerance;
  }

  /** Zero the relative lift encoder only while disabled and physically retracted. */
  public void zeroLiftAtRetractedPosition() {
    if (!DriverStation.isDisabled()) {
      DriverStation.reportWarning("Intake lift calibration is only allowed while disabled.", false);
      return;
    }

    liftStop();
    liftEncoder.setPosition(retractedPosition);
    liftCalibrated = true;
    hasReportedUncalibratedLift = false;
  }

  public boolean isLiftCalibrated() {
    return liftCalibrated;
  }

  private boolean canMoveLift() {
    if (liftCalibrated) {
      return true;
    }

    liftStop();
    if (!hasReportedUncalibratedLift) {
      DriverStation.reportWarning(
          "Intake lift is not calibrated. Physically retract it, then press operator Back before enabling.",
          false);
      hasReportedUncalibratedLift = true;
    }
    return false;
  }
}
