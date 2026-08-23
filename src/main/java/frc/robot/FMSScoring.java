package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.telemetry.ElasticTelemetry;
import java.util.Optional;

/** Publishes 2026 REBUILT hub availability from Driver Station game data. */
public class FMSScoring {
  private boolean isFMSConnected;
  private boolean hubActive;

  public void periodic() {
    isFMSConnected = DriverStation.isFMSAttached();

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    boolean autonomousEnabled = DriverStation.isAutonomousEnabled();
    boolean teleopEnabled = DriverStation.isTeleopEnabled();
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    hubActive = isHubActive(alliance, autonomousEnabled, teleopEnabled, matchTime, gameData);

    ElasticTelemetry.setBoolean("Game/FMS Attached", isFMSConnected);
    ElasticTelemetry.setString("Game/Game Data", gameData.isEmpty() ? "Awaiting data" : gameData);
    ElasticTelemetry.setBoolean("Game/Hub Active", hubActive);
    ElasticTelemetry.setBoolean("Game/CanScoreNow", hubActive);
    ElasticTelemetry.setString(
        "Game/ScoringWindow",
        getHubStatus(alliance, autonomousEnabled, teleopEnabled, matchTime, gameData, hubActive));
    ElasticTelemetry.setString("Game/Score Leader", "Unavailable - no live score feed");
  }

  static boolean isHubActive(
      Optional<DriverStation.Alliance> alliance,
      boolean autonomousEnabled,
      boolean teleopEnabled,
      double matchTime,
      String gameData) {
    if (alliance.isEmpty()) {
      return false;
    }
    if (autonomousEnabled) {
      return true;
    }
    if (!teleopEnabled) {
      return false;
    }

    char inactiveFirst = gameDataCharacter(gameData);
    // Game data is intentionally sent after autonomous. Before then, assume active so a missing
    // Driver Station packet cannot incorrectly block the dashboard state.
    if (inactiveFirst == '\0' || matchTime < 0.0) {
      return true;
    }

    boolean redInactiveFirst = inactiveFirst == 'R';
    boolean shiftOneActive =
        alliance.get() == DriverStation.Alliance.Red ? !redInactiveFirst : redInactiveFirst;

    if (matchTime > 130.0) {
      return true;
    } else if (matchTime > 105.0) {
      return shiftOneActive;
    } else if (matchTime > 80.0) {
      return !shiftOneActive;
    } else if (matchTime > 55.0) {
      return shiftOneActive;
    } else if (matchTime > 30.0) {
      return !shiftOneActive;
    }
    return true;
  }

  private static String getHubStatus(
      Optional<DriverStation.Alliance> alliance,
      boolean autonomousEnabled,
      boolean teleopEnabled,
      double matchTime,
      String gameData,
      boolean active) {
    if (alliance.isEmpty()) {
      return "No Alliance";
    }
    if (autonomousEnabled) {
      return "Autonomous - Hub Active";
    }
    if (!teleopEnabled) {
      return "Robot Disabled";
    }
    if (gameDataCharacter(gameData) == '\0') {
      return "Teleop - Hub assumed active while awaiting game data";
    }
    if (matchTime < 0.0) {
      return "Teleop - Hub assumed active because match time is unavailable";
    }

    return getShiftName(matchTime) + (active ? " - Hub Active" : " - Hub Inactive");
  }

  private static String getShiftName(double matchTime) {
    if (matchTime > 130.0) {
      return "Transition";
    } else if (matchTime > 105.0) {
      return "Shift 1";
    } else if (matchTime > 80.0) {
      return "Shift 2";
    } else if (matchTime > 55.0) {
      return "Shift 3";
    } else if (matchTime > 30.0) {
      return "Shift 4";
    }
    return "Endgame";
  }

  private static char gameDataCharacter(String gameData) {
    if (gameData == null || gameData.isEmpty()) {
      return '\0';
    }

    char value = Character.toUpperCase(gameData.charAt(0));
    return value == 'R' || value == 'B' ? value : '\0';
  }

  public boolean isFMSConnected() {
    return isFMSConnected;
  }

  public boolean canScoreNow() {
    return hubActive;
  }
}
