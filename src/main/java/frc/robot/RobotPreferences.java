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
    public static final SN_BooleanPreference intakeLimitSwitchInvert = new SN_BooleanPreference(
        "intakeLimitSwitchInvert", false);

    // Percent Output
    public static final SN_DoublePreference intakeConeSpeed = new SN_DoublePreference("intakeConeSpeed", 0.5);
    public static final SN_DoublePreference intakeCubeSpeed = new SN_DoublePreference("intakeCubeSpeed", -0.5);
    public static final SN_DoublePreference intakeHoldSpeed = new SN_DoublePreference("intakeHoldSpeed", 0.1);
    public static final SN_DoublePreference intakePlaceConeSpeed = new SN_DoublePreference("intakePlaceConeSpeed",
        -1);
    public static final SN_DoublePreference intakePlaceCubeSpeed = new SN_DoublePreference("intakePlaceCubeSpeed",
        1);
    public static final SN_DoublePreference intakeShootSpeedHigh = new SN_DoublePreference("intakeShootSpeedHigh",
        -0.65);

    // Sator Amps
    public static final SN_DoublePreference intakePieceCollectedBelowAmps = new SN_DoublePreference(
        "intakePieceCollectedBelowAmps", 16);
    public static final SN_DoublePreference intakePieceCollectedAboveAmps = new SN_DoublePreference(
        "intakePieceCollectedAboveAmps", 14);

    // Seconds
    public static final SN_DoublePreference intakePlaceDelay = new SN_DoublePreference("intakePlaceDelay", 0);
  }

  public static final class prefElevator {
    public static final SN_DoublePreference elevatorP = new SN_DoublePreference("elevatorP", 0.1);
    public static final SN_DoublePreference elevatorI = new SN_DoublePreference("elevatorI", 0);
    public static final SN_DoublePreference elevatorD = new SN_DoublePreference("elevatorD", 0);

    public static final SN_DoublePreference elevatorPIDTolerance = new SN_DoublePreference(
        "elevatorPIDTolerance", 0.01);

    public static final SN_DoublePreference elevatorPositionTolerance = new SN_DoublePreference(
        "elevatorPositionTolerance", 0.5);

    // Meters per second
    public static final SN_DoublePreference elevatorMaxVelocity = new SN_DoublePreference("elevatorMaxVelocity", 0.8);
    public static final SN_DoublePreference elevatorMaxAccel = new SN_DoublePreference("elevatorMaxAccel", 0.8);

    // In meters
    public static final SN_DoublePreference elevatorMinPos = new SN_DoublePreference("elevatorMinPos", 0.0);
    public static final SN_DoublePreference elevatorMaxPos = new SN_DoublePreference("elevatorMaxPos", 1.29);
    public static final SN_DoublePreference elevatorIntakeConePos = new SN_DoublePreference("elevatorIntakeConePos",
        0.125);
    public static final SN_DoublePreference elevatorIntakeCubePos = new SN_DoublePreference("elevatorIntakeCubePos",
        0.1);
    public static final SN_DoublePreference elevatorShelf = new SN_DoublePreference("elevatorShelf", 1.26);
    public static final SN_DoublePreference elevatorSingle = new SN_DoublePreference("elevatorSingle", 0.2);
    public static final SN_DoublePreference elevatorStow = new SN_DoublePreference("elevatorStow", 0.15);

    public static final SN_DoublePreference elevatorHybridConeScore = new SN_DoublePreference("elevatorHybridConeScore",
        0.1);
    public static final SN_DoublePreference elevatorHybridCubeScore = new SN_DoublePreference("elevatorHybridCubeScore",
        0.05);
    public static final SN_DoublePreference elevatorMidConeScore = new SN_DoublePreference("elevatorMidConeScore",
        0.547828);
    public static final SN_DoublePreference elevatorMidCubeScore = new SN_DoublePreference("elevatorMidCubeScore",
        0.392799);
    public static final SN_DoublePreference elevatorHighConeScore = new SN_DoublePreference("elevatorHighConeScore",
        1.27);
    public static final SN_DoublePreference elevatorHighCubeScore = new SN_DoublePreference("elevatorHighCubeScore",
        0.998234);

    public static final SN_DoublePreference elevatorEncoderCountsPerMeter = new SN_DoublePreference(
        "elevatorEncoderCountsPerMeter", 36644.718);

  }

  public static final class prefWrist {
    public static final SN_DoublePreference wristF = new SN_DoublePreference("wristF", 0);
    public static final SN_DoublePreference wristP = new SN_DoublePreference("wristP", 0.1);
    public static final SN_DoublePreference wristI = new SN_DoublePreference("wristI", 0);
    public static final SN_DoublePreference wristD = new SN_DoublePreference("wristD", 0);

    // In Degrees
    public static final SN_DoublePreference wristMaxPos = new SN_DoublePreference("wristMaxPos", 238);
    public static final SN_DoublePreference wristMinPos = new SN_DoublePreference("wristMinPos", 2);

    public static final SN_DoublePreference wristPIDTolerance = new SN_DoublePreference("wristPIDTolerance",
        1);
    public static final SN_DoublePreference wristPositionTolerance = new SN_DoublePreference("wristPositionTolerance",
        3);

    public static final SN_DoublePreference wristIntakeAngle = new SN_DoublePreference("wristIntakeAngle", 150);
    public static final SN_DoublePreference wristShelfAngle = new SN_DoublePreference("wristShelfAngle", 145);
    public static final SN_DoublePreference wristSingleAngle = new SN_DoublePreference("wristSingleAngle", 105);

    public static final SN_DoublePreference wristStowAngle = new SN_DoublePreference("wristStowAngle", 44);

    public static final SN_DoublePreference wristScoreHybridCubeAngle = new SN_DoublePreference(
        "wristScoreHybridCubeAngle", 117.776507);
    public static final SN_DoublePreference wristScoreMidConeAngle = new SN_DoublePreference("wristScoreMidConeAngle",
        107);
    public static final SN_DoublePreference wristScoreHighConeAngle = new SN_DoublePreference("wristScoreHighAngle",
        150);
    public static final SN_DoublePreference wristScoreHighCubeAngle = new SN_DoublePreference("wristScoreHighCubeAngle",
        98.466439);

    // Degrees Per Second
    public static final SN_DoublePreference wristMaxVelocity = new SN_DoublePreference("wristMaxVelocity", 75);
    public static final SN_DoublePreference wristMaxAccel = new SN_DoublePreference("wristMaxAccel", 75);

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
