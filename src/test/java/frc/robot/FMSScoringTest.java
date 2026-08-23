package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class FMSScoringTest {
  @Test
  void hubAvailabilityFollowsThe2026ShiftSchedule() {
    assertFalse(FMSScoring.isHubActive(Optional.empty(), true, true, 140.0, "R"));
    assertTrue(
        FMSScoring.isHubActive(Optional.of(DriverStation.Alliance.Red), true, false, 140.0, "R"));
    assertFalse(
        FMSScoring.isHubActive(Optional.of(DriverStation.Alliance.Red), false, false, 140.0, "R"));

    // Red goes inactive first: Blue owns shifts 1 and 3, Red owns shifts 2 and 4.
    assertFalse(
        FMSScoring.isHubActive(Optional.of(DriverStation.Alliance.Red), false, true, 120.0, "R"));
    assertTrue(
        FMSScoring.isHubActive(Optional.of(DriverStation.Alliance.Blue), false, true, 120.0, "R"));
    assertTrue(
        FMSScoring.isHubActive(Optional.of(DriverStation.Alliance.Red), false, true, 90.0, "R"));
    assertFalse(
        FMSScoring.isHubActive(Optional.of(DriverStation.Alliance.Red), false, true, 65.0, "R"));
    assertTrue(
        FMSScoring.isHubActive(Optional.of(DriverStation.Alliance.Red), false, true, 40.0, "R"));
    assertTrue(
        FMSScoring.isHubActive(Optional.of(DriverStation.Alliance.Red), false, true, 20.0, "R"));
  }

  @Test
  void missingOrInvalidGameDataFailsOpenDuringTeleop() {
    Optional<DriverStation.Alliance> red = Optional.of(DriverStation.Alliance.Red);

    assertTrue(FMSScoring.isHubActive(red, false, true, 120.0, ""));
    assertTrue(FMSScoring.isHubActive(red, false, true, 120.0, "invalid"));
    assertTrue(FMSScoring.isHubActive(red, false, true, -1.0, "R"));
  }
}
