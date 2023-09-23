package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.util.Units;

public class RobotPreferences {

  public static final boolean useNetworkTables = false;

  // Order of Subsystems: The order that the robots inputs go, starting from
  // controllers

  public static final class prefControllers {
    public static final SN_DoublePreference rumbleOutput = new SN_DoublePreference("rumbleOutput", 0.2);
    public static final SN_DoublePreference rumbleDelay = new SN_DoublePreference("rumbleDelay", 0.5);
  }

  public static final class prefDrivetrain {

    public static final SN_DoublePreference driveF = new SN_DoublePreference("driveF", 0.045);
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.1);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0.0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 1.0);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 0.3);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0.0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 6.0);

    // percent of max module speed that is required for the module to steer
    // (a stopped wheel shouldn't steer)
    public static final SN_DoublePreference percentToSteer = new SN_DoublePreference("percentToSteer", 0.01);

    // max translational speed in feet per second while driving using a controller
    // 16.3 FPS is maximum due to gearing
    public static final SN_DoublePreference driveSpeed = new SN_DoublePreference("driveSpeed", 16.3);

    // max rotational speed in degrees per second while driving using a controller
    // 943.751 DPS is maximum due to gearing and robot size
    public static final SN_DoublePreference turnSpeed = new SN_DoublePreference("turnSpeed", 360);

    // Value to multiply with translation velocity when trigger is all the way held
    // down.
    public static final SN_DoublePreference triggerValue = new SN_DoublePreference("triggerValue", .2);

    public static final SN_DoublePreference autoThetaP = new SN_DoublePreference("autoThetaP", 0.5);
    public static final SN_DoublePreference autoThetaI = new SN_DoublePreference("autoThetaI", 0.0);
    public static final SN_DoublePreference autoThetaD = new SN_DoublePreference("autoThetaD", 0.0);

    public static final SN_DoublePreference autoTransP = new SN_DoublePreference("autoTransP", 2);
    public static final SN_DoublePreference autoTransI = new SN_DoublePreference("autoTransI", 0);
    public static final SN_DoublePreference autoTransD = new SN_DoublePreference("autoTransD", 0);

    public static final SN_DoublePreference autoMaxSpeedFeet = new SN_DoublePreference(
        "autoMaxSpeedFeet", 2);

    public static final SN_DoublePreference autoMaxAccelFeet = new SN_DoublePreference(
        "autoMaxAccelFeet", 1);

    public static final SN_DoublePreference fasterAutoMaxSpeedFeet = new SN_DoublePreference(
        "autoMaxSpeedFeet", 5);

    public static final SN_DoublePreference fasterAutoMaxAccelFeet = new SN_DoublePreference(
        "autoMaxAccelFeet", 4);

    public static final SN_DoublePreference teleTransP = new SN_DoublePreference("teleTransP", 0);
    public static final SN_DoublePreference teleTransI = new SN_DoublePreference("teleTransI", 0);
    public static final SN_DoublePreference teleTransD = new SN_DoublePreference("teleTransD", 0);
    // feet per second
    public static final SN_DoublePreference teleTransMaxSpeed = new SN_DoublePreference("teleTransMaxSpeed", 16.3);
    // feet per second per second
    public static final SN_DoublePreference teleTransMaxAccel = new SN_DoublePreference("teleTransMaxAccel", 5);
    // inches
    public static final SN_DoublePreference teleTransTolerance = new SN_DoublePreference("teleTransTolerance", 1);

    public static final SN_DoublePreference teleThetaP = new SN_DoublePreference("teleThetaP", 8.0);
    public static final SN_DoublePreference teleThetaI = new SN_DoublePreference("teleThetaI", 0);
    public static final SN_DoublePreference teleThetaD = new SN_DoublePreference("teleThetaD", 0.2);
    // degrees per second
    public static final SN_DoublePreference teleThetaMaxSpeed = new SN_DoublePreference("teleThetaMaxSpeed", 270);
    // degrees
    public static final SN_DoublePreference teleThetaTolerance = new SN_DoublePreference("teleThetaTolerance", 2);

    // degrees
    public static final SN_DoublePreference tiltedThreshold = new SN_DoublePreference("tiltedThreshold", 5);

    // feet per second
    public static final SN_DoublePreference dockingSpeed = new SN_DoublePreference("dockingSpeed", 0.05);

    // current limiting (values taken from BaseFalconSwerve)
    public static final SN_BooleanPreference driveEnableCurrentLimit = new SN_BooleanPreference(
        "driveEnableCurrentLimit", true);
    public static final SN_DoublePreference driveHoldingCurrentLimit = new SN_DoublePreference(
        "driveHoldingCurrentLimit", 35);
    public static final SN_DoublePreference drivePeakCurrentLimit = new SN_DoublePreference(
        "drivePeakCurrentLimit", 60);
    public static final SN_DoublePreference drivePeakCurrentTime = new SN_DoublePreference(
        "drivePeakCurrentTime", 0.1);

    public static final SN_BooleanPreference steerEnableCurrentLimit = new SN_BooleanPreference(
        "steerEnableCurrentLimit", true);
    public static final SN_DoublePreference steerHoldingCurrentLimit = new SN_DoublePreference(
        "steerHoldingCurrentLimit", 25);
    public static final SN_DoublePreference steerPeakCurrentLimit = new SN_DoublePreference(
        "steerPeakCurrentLimit", 40);
    public static final SN_DoublePreference steerPeakCurrentTime = new SN_DoublePreference(
        "steerPeakCurrentTime", 0.1);

    public static final SN_DoublePreference measurementStdDevsFeet = new SN_DoublePreference(
        "measurementStdDevsFeet", Units.metersToFeet(0.1));
    public static final SN_DoublePreference measurementStdDevsDegrees = new SN_DoublePreference(
        "measurementStdDevsDegrees", Units.radiansToDegrees(0.1));
  }

  public static final class prefIntake {
    // Percent Output
    public static final SN_DoublePreference intakeConeSpeed = new SN_DoublePreference("intakeConeSpeed", 0.5);
    public static final SN_DoublePreference intakeCubeSpeed = new SN_DoublePreference("intakeCubeSpeed", -0.5);
    public static final SN_DoublePreference intakeHoldSpeed = new SN_DoublePreference("intakeHoldSpeed", 0.1);
    public static final SN_DoublePreference intakePlaceConeSpeed = new SN_DoublePreference("intakePlaceConeSpeed",
        -0.25);
    public static final SN_DoublePreference intakePlaceCubeSpeed = new SN_DoublePreference("intakePlaceCubeSpeed",
        0.25);
    public static final SN_DoublePreference intakeShootSpeedHigh = new SN_DoublePreference("intakeShootSpeedHigh",
        -0.65);
  }

  public static final class prefElevator {
    public static final SN_DoublePreference elevatorF = new SN_DoublePreference("elevatorF", 0);
    public static final SN_DoublePreference elevatorP = new SN_DoublePreference("elevatorP", 0);
    public static final SN_DoublePreference elevatorI = new SN_DoublePreference("elevatorI", 0);
    public static final SN_DoublePreference elevatorD = new SN_DoublePreference("elevatorD", 0);

    // In Encoder Ticks
    // TODO: Find actual values please
    public static final SN_DoublePreference elevatorMaxPos = new SN_DoublePreference("elevatorMaxPos", 10);
    public static final SN_DoublePreference elevatorMinPos = new SN_DoublePreference("elevatorMinPos", 0);
    public static final SN_DoublePreference elevatorIntakingPos = new SN_DoublePreference("elevatorIntakingPos",
        2);
    public static final SN_DoublePreference elevatorStow = new SN_DoublePreference("elevatorStow", 0);
    public static final SN_DoublePreference elevatorPositionTolerance = new SN_DoublePreference(
        "elevatorPositionTolerance", 1);
    public static final SN_DoublePreference hybridScore = new SN_DoublePreference("hybridConeScore",
        0);
    public static final SN_DoublePreference midConeScore = new SN_DoublePreference("midConeScore", 1);
    public static final SN_DoublePreference highConeScore = new SN_DoublePreference("highConeScore", 2);
    public static final SN_DoublePreference midCubeScore = new SN_DoublePreference("midCubeScore", 1);
    public static final SN_DoublePreference highCubeScore = new SN_DoublePreference("highCubeScore", 2);

    // stole this value from 2022 drivetrain preferences, will need to change after
    // we get more information on the new design
    // TODO: Change this to real values
    public static final SN_DoublePreference elevatorEncoderCountsPerFoot = new SN_DoublePreference(
        "elevatorEncoderCountsPerFoot", 11734);
  }

  public static final class prefWrist {
    public static final SN_DoublePreference wristF = new SN_DoublePreference("wristF", 0);
    public static final SN_DoublePreference wristP = new SN_DoublePreference("wristP", 0);
    public static final SN_DoublePreference wristI = new SN_DoublePreference("wristI", 0);
    public static final SN_DoublePreference wristD = new SN_DoublePreference("wristD", 0);

    // In Encoder Ticks
    public static final SN_DoublePreference wristMaxPos = new SN_DoublePreference("WristMaxPos", 10);
    public static final SN_DoublePreference wristMinPos = new SN_DoublePreference("WristMinPos", 0);

    public static final SN_DoublePreference wristIntakingAngle = new SN_DoublePreference("WristIntakingAngle", 90);
    public static final SN_DoublePreference wristStowAngle = new SN_DoublePreference("WristStowAngle", 0);
    public static final SN_DoublePreference wristScoringAngle = new SN_DoublePreference("wristScoringAngle", 0);
  }

  public static final class prefVision {

    public static final SN_DoublePreference measurementStdDevsFeet = new SN_DoublePreference(
        "measurementStdDevsFeet", Units.metersToFeet(0.9));
    public static final SN_DoublePreference measurementStdDevsDegrees = new SN_DoublePreference(
        "measurementStdDevsDegrees", Units.radiansToDegrees(0.9));

    public static final SN_DoublePreference chargeStationCenterX = new SN_DoublePreference("chargeStationCenterX", 3.9);
    public static final SN_DoublePreference chargeStationCenterToleranceX = new SN_DoublePreference(
        "chargeStationCenterToleranceX", 0.1);
    public static final SN_DoublePreference chargeStationCenterY = new SN_DoublePreference("chargeStationCenterY",
        2.75);
    public static final SN_DoublePreference chargeStationCenterToleranceY = new SN_DoublePreference(
        "chargeStationCenterToleranceY", 1.18);

    public static final SN_DoublePreference gridAlignmentToleranceY = new SN_DoublePreference("gridAlignmentToleranceY",
        0.05);
    public static final SN_DoublePreference gridLEDsXPosMaxBlue = new SN_DoublePreference("gridLEDsXPosMaxBlue", 2.5);
    public static final SN_DoublePreference gridLEDsXPosMaxRed = new SN_DoublePreference("gridLEDsXPosMaxRed", 13.5);

  }

  public static final class prefLEDs {
    public static final SN_DoublePreference timeChargeStationLEDsOn = new SN_DoublePreference("timeChargeStationLEDsOn",
        30);
  }

}
