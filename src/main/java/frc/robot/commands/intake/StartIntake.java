package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

/** Starts the intake roller and leaves it running until another intake command stops it. */
public class StartIntake extends InstantCommand {
  public StartIntake(Intake intake) {
    super(intake::intake, intake);
  }
}
