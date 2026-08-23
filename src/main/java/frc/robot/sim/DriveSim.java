package frc.robot.sim;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.MAXSwerveModule;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/**
 * The maple-sim wrapper for the drivetrain.
 *
 * <p>Uses the "simple approach" ({@link SelfControlledSwerveDriveSimulation}) which runs its own
 * internal closed-loop on the simulated motors. Each update cycle:
 *
 * <ol>
 *   <li>Reads the desired module states from the real {@link MAXSwerveModule} objects.
 *   <li>Feeds them into the self-controlled simulation.
 *   <li>Steps the physics engine forward.
 *   <li>Writes the resulting encoder and gyro values back into the WPILib sim objects so the
 *       student code's odometry "sees" realistic, slipping wheels.
 * </ol>
 */
public class DriveSim {
  private final MAXSwerveModule[] m_modules;
  private final Pigeon2 m_gyro;

  private final SelfControlledSwerveDriveSimulation m_simDrive;

  public DriveSim(
      MAXSwerveModule frontLeft,
      MAXSwerveModule frontRight,
      MAXSwerveModule rearLeft,
      MAXSwerveModule rearRight,
      Pigeon2 gyro) {
    m_modules = new MAXSwerveModule[] {frontLeft, frontRight, rearLeft, rearRight};
    m_gyro = gyro;

    // Set the arena to the 2026 REBUILT field
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));

    // Create the underlying physics simulation
    SwerveDriveSimulation swerveSim =
        new SwerveDriveSimulation(
            SimConstants.createDriveTrainConfig(), SimConstants.kStartingPose);

    // Wrap it with the self-controlled layer
    m_simDrive = new SelfControlledSwerveDriveSimulation(swerveSim);

    // Register with the simulated arena so physics ticks happen
    SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim);
  }

  /** Steps the simulation forward. Called once per robot periodic (20 ms). */
  @SuppressWarnings(
      "deprecation") // Maple exposes its simulated gyro reading through a deprecated API.
  public void update() {
    // 1. Read the desired module states from the real motor objects
    SwerveModuleState[] desiredStates = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      desiredStates[i] = m_modules[i].getDesiredState();
    }

    // 2. Feed the desired states into the self-controlled sim
    // (it runs its own PID on the simulated motors)
    m_simDrive.runSwerveStates(desiredStates);

    // 3. Step the entire simulated arena forward (physics, collisions, etc.)
    SimulatedArena.getInstance().simulationPeriodic();

    // 4. Update the self-controlled sim's internal odometry
    m_simDrive.periodic();

    // 5. Feed the simulated sensors back into the exact motor controllers and gyro that the
    // robot code reads. This lets robot-side odometry and path following use the simulated wheel
    // slip/noise instead of a separate ground-truth pose.
    SwerveModulePosition[] modulePositions = m_simDrive.getLatestModulePositions();
    SwerveModuleState[] measuredStates = m_simDrive.getMeasuredStates();
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setSimState(modulePositions[i], measuredStates[i]);
    }
    m_gyro.getSimState().setRawYaw(m_simDrive.getRawGyroAngle().getDegrees());
  }

  /**
   * @return Actual Pose (Ground Truth) managed entirely by maple-sim. The student code must never
   *     have access to this pose directly.
   */
  public Pose2d getActualPose() {
    return m_simDrive.getActualPoseInSimulationWorld();
  }

  /**
   * @return The odometry-estimated pose from the sim's internal pose estimator.
   */
  public Pose2d getOdometryPose() {
    return m_simDrive.getOdometryEstimatedPose();
  }

  /** Instantly teleports the simulated robot to a new pose on the field. */
  public void setSimulationWorldPose(Pose2d pose) {
    // Depending on maple-sim version, we either set it on the wrapper or the underlying sim
    m_simDrive.setSimulationWorldPose(pose);
  }
}
