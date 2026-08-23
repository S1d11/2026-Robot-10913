package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.telemetry.ElasticTelemetry;

public class FMSScoring {
  private final NetworkTable fmsTable;
  private boolean isFMSConnected = false;

  public FMSScoring() {
    fmsTable = NetworkTableInstance.getDefault().getTable("FMSInfo");
  }

  public void periodic() {
    isFMSConnected = DriverStation.isFMSAttached();
    updateScoringWindow();

    if (!isFMSConnected) {
      ElasticTelemetry.setString("Game/Score Leader", "N/A - No FMS");
      return;
    }

    updateScores();
  }

  private void updateScores() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      ElasticTelemetry.setString("Game/Score Leader", "Unavailable - No Alliance");
      return;
    }

    boolean isRed = alliance.get() == DriverStation.Alliance.Red;

    var ourScoreEntry = fmsTable.getEntry(isRed ? "RedScore" : "BlueScore");
    var opponentScoreEntry = fmsTable.getEntry(isRed ? "BlueScore" : "RedScore");
    if (!ourScoreEntry.exists() || !opponentScoreEntry.exists()) {
      ElasticTelemetry.setString("Game/Score Leader", "Unavailable - Score Data Missing");
      return;
    }

    int ourScore = (int) ourScoreEntry.getDouble(0);
    int opponentScore = (int) opponentScoreEntry.getDouble(0);

    ElasticTelemetry.setNumber("Game/OurScore", ourScore);
    ElasticTelemetry.setNumber("Game/OpponentScore", opponentScore);

    String scoreLeader;
    if (ourScore > opponentScore) {
      scoreLeader = "Us (" + ourScore + " pts)";
    } else if (opponentScore > ourScore) {
      scoreLeader = "Opponent (" + opponentScore + " pts)";
    } else if (ourScore == 0 && opponentScore == 0) {
      scoreLeader = "N/A";
    } else {
      scoreLeader = "Tie (" + ourScore + " pts)";
    }

    ElasticTelemetry.setString("Game/Score Leader", scoreLeader);
  }

  private void updateScoringWindow() {
    boolean canScore = isFMSConnected && (DriverStation.isAutonomous() || DriverStation.isTeleop());
    String reason;

    if (!isFMSConnected) {
      reason = "No FMS Attached";
    } else if (DriverStation.isDisabled()) {
      reason = "Robot Disabled";
    } else if (DriverStation.isAutonomous()) {
      reason = "Autonomous Enabled";
    } else if (DriverStation.isTeleop()) {
      reason = "Teleoperated Enabled";
    } else {
      reason = "Not a Scoring Period";
    }

    ElasticTelemetry.setBoolean("Game/CanScoreNow", canScore);
    ElasticTelemetry.setString("Game/ScoringWindow", reason);
  }

  public boolean isFMSConnected() {
    return isFMSConnected;
  }

  public boolean canScoreNow() {
    return isFMSConnected && (DriverStation.isAutonomous() || DriverStation.isTeleop());
  }
}
