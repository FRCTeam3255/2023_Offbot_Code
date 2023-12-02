// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap.mapDrivetrain;

public final class Constants {

  public static final boolean OUTPUT_DEBUG_VALUES = true;
  public static final boolean SILENCE_JOYSTICK_WARNINGS = true;

  // TODO: Update with Actual ports
  public static final String[] PDH_DEVICES = {
      "Swerve FR Drive", "Swerve FR Steer",
      null, null, null, null, null, null,
      "Swerve FL Drive", "Swerve FL Steer",
      "Swerve BL Drive", "Swerve BL Steer",
      null, null, null, null, null, null,
      "Swerve BR Drive", "Swerve BR Steer",
      null, null, null, null };

  // Order of Subsystems: The order that the robots inputs go, starting from
  // controllers

  public static final class constControllers {
    public static final double DRIVER_LEFT_STICK_X_DEADBAND = 0.05;
  }

  // Drivetrain (no subclass)

  // note: these were physically measured center to center of the wheel on a
  // 29"x29" drivetrain with MK4i's.
  public static final double TRACK_WIDTH = Units.inchesToMeters(23.75);
  public static final double WHEELBASE = Units.inchesToMeters(23.75);

  // Swerve Modules

  /*
   * Order of modules:
   * 
   * 0: Front Left
   * 1: Front Right
   * 2: Back Left
   * 3: Back Right
   * 
   * 0 1
   * 2 3
   */

  // Colson Wheels
  private static final double WHEEL_DIAMETER = Units.inchesToMeters(3.8);
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

  // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  // L3 gearing, Falcon drive motor
  public static final double DRIVE_GEAR_RATIO = 6.12;
  public static final double STEER_GEAR_RATIO = 150.0 / 7.0;
  public static final double MAX_MODULE_SPEED = Units.feetToMeters(16.3);

  public static final boolean DRIVE_MOTOR_INVERT = false;
  public static final boolean STEER_MOTOR_INVERT = true;

  public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
  public static final NeutralMode STEER_NEUTRAL_MODE = NeutralMode.Coast;

  public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 16.171875;
  public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 232.119140625;
  public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 87.36328125;
  public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 332.9296875;

  // module positions follow the WPILib robot coordinate system
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
  public static final SN_SwerveModuleConstants MODULE_0 = new SN_SwerveModuleConstants(
      mapDrivetrain.FRONT_LEFT_DRIVE_CAN,
      mapDrivetrain.FRONT_LEFT_STEER_CAN,
      mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN,
      FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          WHEELBASE / 2.0,
          TRACK_WIDTH / 2.0),
      0);

  public static final SN_SwerveModuleConstants MODULE_1 = new SN_SwerveModuleConstants(
      mapDrivetrain.FRONT_RIGHT_DRIVE_CAN,
      mapDrivetrain.FRONT_RIGHT_STEER_CAN,
      mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN,
      FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          WHEELBASE / 2.0,
          -TRACK_WIDTH / 2.0),
      1);

  public static final SN_SwerveModuleConstants MODULE_2 = new SN_SwerveModuleConstants(
      mapDrivetrain.BACK_LEFT_DRIVE_CAN,
      mapDrivetrain.BACK_LEFT_STEER_CAN,
      mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN,
      BACK_LEFT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          -WHEELBASE / 2.0,
          TRACK_WIDTH / 2.0),
      2);

  public static final SN_SwerveModuleConstants MODULE_3 = new SN_SwerveModuleConstants(
      mapDrivetrain.BACK_RIGHT_DRIVE_CAN,
      mapDrivetrain.BACK_RIGHT_STEER_CAN,
      mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN,
      BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          -WHEELBASE / 2.0,
          -TRACK_WIDTH / 2.0),
      3);

  public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      MODULE_0.position,
      MODULE_1.position,
      MODULE_2.position,
      MODULE_3.position);

  public static final double PRAC_FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 114.521484;
  public static final double PRAC_FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 341.806641;
  public static final double PRAC_BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 289.511719;
  public static final double PRAC_BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 245.566406;

  // module positions follow the WPILib robot coordinate system
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
  public static final SN_SwerveModuleConstants PRAC_MODULE_0 = new SN_SwerveModuleConstants(
      mapDrivetrain.FRONT_LEFT_DRIVE_CAN,
      mapDrivetrain.FRONT_LEFT_STEER_CAN,
      mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN,
      PRAC_FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          WHEELBASE / 2.0,
          TRACK_WIDTH / 2.0),
      0);

  public static final SN_SwerveModuleConstants PRAC_MODULE_1 = new SN_SwerveModuleConstants(
      mapDrivetrain.FRONT_RIGHT_DRIVE_CAN,
      mapDrivetrain.FRONT_RIGHT_STEER_CAN,
      mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN,
      PRAC_FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          WHEELBASE / 2.0,
          -TRACK_WIDTH / 2.0),
      1);

  public static final SN_SwerveModuleConstants PRAC_MODULE_2 = new SN_SwerveModuleConstants(
      mapDrivetrain.BACK_LEFT_DRIVE_CAN,
      mapDrivetrain.BACK_LEFT_STEER_CAN,
      mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN,
      PRAC_BACK_LEFT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          -WHEELBASE / 2.0,
          TRACK_WIDTH / 2.0),
      2);

  public static final SN_SwerveModuleConstants PRAC_MODULE_3 = new SN_SwerveModuleConstants(
      mapDrivetrain.BACK_RIGHT_DRIVE_CAN,
      mapDrivetrain.BACK_RIGHT_STEER_CAN,
      mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN,
      PRAC_BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET, // absolute encoder offset
      new Translation2d(
          -WHEELBASE / 2.0,
          -TRACK_WIDTH / 2.0),
      3);

  public static final SwerveDriveKinematics PRAC_SWERVE_KINEMATICS = new SwerveDriveKinematics(
      PRAC_MODULE_0.position,
      PRAC_MODULE_1.position,
      PRAC_MODULE_2.position,
      PRAC_MODULE_3.position);

  public static final Rotation2d MODULE_0_DEFENSE_ANGLE = Rotation2d.fromDegrees(45);
  public static final Rotation2d MODULE_1_DEFENSE_ANGLE = Rotation2d.fromDegrees(135);
  public static final Rotation2d MODULE_2_DEFENSE_ANGLE = Rotation2d.fromDegrees(135);
  public static final Rotation2d MODULE_3_DEFENSE_ANGLE = Rotation2d.fromDegrees(45);

  // end drivetrain section

  public static final class constIntake {
    public static final boolean MOTOR_INVERTED = true;

    public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;

    public static final double CURRENT_LIMIT_FLOOR_AMPS = 8; // Floor: what we limit it to
    public static final double CURRENT_LIMIT_CEILING_AMPS = 40; // Ceiling: when we begin limiting
    public static final double CURRENT_LIMIT_AFTER_SEC = 10;
  }

  public static final class constWrist {
    public static final double CURRENT_LIMIT_FLOOR_AMPS = 1; // Floor: what we limit it to
    public static final double CURRENT_LIMIT_CEILING_AMPS = 30; // Ceiling: when we begin limiting
    public static final double CURRENT_LIMIT_AFTER_SEC = 0.01;

    public static final double ABSOLUTE_ENCODER_OFFSET = 0.333;
    // The value in which the Raw Absolute Encoder value rolls over from 0 to 1
    public static final double ABSOLUTE_ENCODER_ROLLOVER_OFFSET = 0.667530;
    public static final boolean ABSOLUTE_ENCODER_INVERT = true;

    public static final double GEAR_RATIO = 40.09;

  }

  public static final class constElevator {
    public static final double CURRENT_LIMIT_FLOOR_AMPS = 1; // Floor: what we limit it to
    public static final double CURRENT_LIMIT_CEILING_AMPS = 100; // Ceiling: when we begin limiting
    public static final double CURRENT_LIMIT_AFTER_SEC = 0.01;

    public static final double ABSOLUTE_ENCODER_OFFSET = 0.840862;
    public static final double ABSOLUTE_ENCODER_ROLLOVER_OFFSET = 0.159017;
    public static final boolean ABSOLUTE_ENCODER_INVERT = true;
    public static final boolean INVERT_LEFT_MOTOR = true;

    public static final double CIRCUMFERENCE = 0.13972;
    public static final double GEAR_RATIO = 2.5;
    public static final double ANGLE_TO_BASE_DEGREES = 54.54;

  }

  public static final class constVision {
    public static final String LIFECAM_PHOTON_NAME = "Microsoft_LifeCam_HD-3000";
    public static final String AR_PHOTON_NAME = "Global_Shutter_Camera";
    public static final String OV_PHOTON_NAME = "Arducam_OV9281_USB_Camera";

    public static final Transform3d ROBOT_TO_OV = new Transform3d(
        new Translation3d(Units.inchesToMeters(-3), Units.inchesToMeters(6.5), 0.46355),
        new Rotation3d(0, 0, 0));
    public static final Transform3d ROBOT_TO_AR = new Transform3d(
        new Translation3d(Units.inchesToMeters(-3.6875), Units.inchesToMeters(6.5), 0.46355),
        new Rotation3d(0, 0, Units.degreesToRadians(180)));
    public static final Transform3d ROBOT_TO_LIFECAM = new Transform3d(new Translation3d(0.4191, -0.1905, 0.6604),
        new Rotation3d(0, 0, 0));
  }

  public static final class constLEDs {
    public static final int LED_NUMBER = 150; // TODO: Get actual value
    public static final int LED_BRIGHTNESS = 1;

    public static final int[] HAS_CUBE_COLOR = { 175, 18, 236 };
    public static final int[] HAS_CONE_COLOR = { 255, 238, 30 };

    public static final StrobeAnimation INTAKING_CUBE_COLOR = new StrobeAnimation(175, 18, 236, 0, 0.1, LED_NUMBER);
    public static final StrobeAnimation INTAKING_CONE_COLOR = new StrobeAnimation(255, 238, 30, 0, 0.1, LED_NUMBER);

    public static final int[] FAILURE_COLOR = { 255, 0, 0 };

    public static final int[] DEFAULT_COLOR = { 0, 0, 0 };

    public static final RainbowAnimation DEFENSE_MODE_ANIMATION = new RainbowAnimation(LED_BRIGHTNESS, 0.8,
        LED_NUMBER);
  }

  public enum GamePiece {
    NONE, CUBE, CONE, HUH
  }

  public enum DesiredHeight {
    NONE, HYBRID, MID, HIGH
  }
}
