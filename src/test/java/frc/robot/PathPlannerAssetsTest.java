package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ModuleConstants;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

class PathPlannerAssetsTest {
  private static final Path PATHPLANNER_DIRECTORY = Path.of("src/main/deploy/pathplanner");

  @Test
  void eventMarkersContainCommands() throws IOException {
    try (Stream<Path> paths = Files.list(PATHPLANNER_DIRECTORY.resolve("paths"))) {
      for (Path path : paths.filter(file -> file.toString().endsWith(".path")).toList()) {
        assertFalse(
            Files.readString(path).contains("\"command\": null"),
            () -> path + " contains an inert event marker");
      }
    }
  }

  @Test
  void settingsMatchTheDrivetrainKinematics() throws IOException {
    String settings = Files.readString(PATHPLANNER_DIRECTORY.resolve("settings.json"));
    double halfTrack = DriveConstants.kTrackWidth / 2.0;

    assertEquals(DriveConstants.kBumperWidthMeters, getNumber(settings, "robotWidth"), 1e-9);
    assertEquals(DriveConstants.kBumperLengthMeters, getNumber(settings, "robotLength"), 1e-9);
    assertEquals(DriveConstants.kRobotMassKilograms, getNumber(settings, "robotMass"), 1e-9);
    assertEquals(
        DriveConstants.kRobotMomentOfInertiaKgMetersSquared, getNumber(settings, "robotMOI"), 1e-9);
    assertEquals(DriveConstants.kTrackWidth, getNumber(settings, "robotTrackwidth"), 1e-9);
    assertEquals(halfTrack, getNumber(settings, "flModuleX"), 1e-9);
    assertEquals(halfTrack, getNumber(settings, "flModuleY"), 1e-9);
    assertEquals(-halfTrack, getNumber(settings, "brModuleX"), 1e-9);
    assertEquals(-halfTrack, getNumber(settings, "brModuleY"), 1e-9);
    assertEquals(ModuleConstants.kDrivingMotorReduction, getNumber(settings, "driveGearing"), 1e-9);
    assertEquals(
        DriveConstants.kMaxSpeedMetersPerSecond, getNumber(settings, "maxDriveSpeed"), 1e-9);
    assertEquals("NEO", getString(settings, "driveMotorType"));
  }

  @Test
  void hubTargetsAreAllianceMirrors() {
    assertEquals(
        FieldConstants.kFieldLength,
        FieldConstants.kBlueHub.getX() + FieldConstants.kRedHub.getX(),
        1e-9);
    assertEquals(FieldConstants.kBlueHub.getY(), FieldConstants.kRedHub.getY(), 1e-9);
    assertTrue(FieldConstants.kBlueHub.getX() < FieldConstants.kRedHub.getX());
  }

  private static double getNumber(String json, String key) {
    Pattern pattern =
        Pattern.compile("\\\"" + Pattern.quote(key) + "\\\"\\s*:\\s*(-?\\d+(?:\\.\\d+)?)");
    Matcher matcher = pattern.matcher(json);
    assertTrue(matcher.find(), () -> "Missing numeric key: " + key);
    return Double.parseDouble(matcher.group(1));
  }

  private static String getString(String json, String key) {
    Pattern pattern =
        Pattern.compile("\\\"" + Pattern.quote(key) + "\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");
    Matcher matcher = pattern.matcher(json);
    assertTrue(matcher.find(), () -> "Missing string key: " + key);
    return matcher.group(1);
  }
}
