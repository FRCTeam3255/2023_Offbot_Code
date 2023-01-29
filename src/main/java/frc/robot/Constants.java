// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap.mapDrivetrain;

public final class Constants {

  public static final class constVision {
    public static final String lifecamPhotonName = "Microsoft_LifeCam_HD-3000";
    public static final String ARPhotonName = "Global_Shutter_Camera";
    public static final String OVPhotonName = "Arducam_OV9281_USB_Camera";

    public static final Transform3d robotToAR = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d robotToOV = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d robotToLifecam = new Transform3d(new Translation3d(0.4191, -0.1905, 0.6604),
        new Rotation3d(0, 0, 0));
  }

  public enum GamePiece {
    NONE, CUBE, CONE, HUH
  }

  public static final boolean OUTPUT_DEBUG_VALUES = false;

  // NEO (big and small) encoder counts per rotation
  public static final double NEO_ENCODER_CPR = 42;

  // Drivetrain (no subclass)

  // note: these were physically measured center to center of the wheel on a
  // 29"x29" drivetrain with MK4i's. Ideally these are
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
   *
   * Like reading
   * four words
   */

  // tread is 11 inches long
  // 11 / 3.14 = 3.50 inch wheel (w/o tread) diameter
  // tread is 0.3 inches thick
  // 3.5 + 0.3 = 3.8 inch wheel (w/ tread) diameter
  private static final double WHEEL_DIAMETER = Units.inchesToMeters(3.8);
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

  // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  // L2 gearing, Falcon drive motor
  public static final double DRIVE_GEAR_RATIO = 6.75;
  public static final double STEER_GEAR_RATIO = 150.0 / 7.0;
  public static final double MAX_MODULE_SPEED = Units.feetToMeters(16.3);

  public static final boolean DRIVE_MOTOR_INVERT = false;
  public static final boolean STEER_MOTOR_INVERT = true;

  public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
  public static final NeutralMode STEER_NEUTRAL_MODE = NeutralMode.Coast;

  // module positions follow the WPILib robot coordinate system
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
  public static final SN_SwerveModuleConstants MODULE_0 = new SN_SwerveModuleConstants(
      mapDrivetrain.FRONT_LEFT_DRIVE_CAN,
      mapDrivetrain.FRONT_LEFT_STEER_CAN,
      mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN,
      70.927734, // absolute encoder offset
      new Translation2d(
          WHEELBASE / 2.0,
          TRACK_WIDTH / 2.0),
      0);

  public static final SN_SwerveModuleConstants MODULE_1 = new SN_SwerveModuleConstants(
      mapDrivetrain.FRONT_RIGHT_DRIVE_CAN,
      mapDrivetrain.FRONT_RIGHT_STEER_CAN,
      mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN,
      24.433594, // absolute encoder offset
      new Translation2d(
          WHEELBASE / 2.0,
          -TRACK_WIDTH / 2.0),
      1);

  public static final SN_SwerveModuleConstants MODULE_2 = new SN_SwerveModuleConstants(
      mapDrivetrain.BACK_LEFT_DRIVE_CAN,
      mapDrivetrain.BACK_LEFT_STEER_CAN,
      mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN,
      331.787109, // absolute encoder offset
      new Translation2d(
          -WHEELBASE / 2.0,
          TRACK_WIDTH / 2.0),
      2);

  public static final SN_SwerveModuleConstants MODULE_3 = new SN_SwerveModuleConstants(
      mapDrivetrain.BACK_RIGHT_DRIVE_CAN,
      mapDrivetrain.BACK_RIGHT_STEER_CAN,
      mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN,
      66.005859, // absolute encoder offset
      new Translation2d(
          -WHEELBASE / 2.0,
          -TRACK_WIDTH / 2.0),
      3);

  public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      MODULE_0.position,
      MODULE_1.position,
      MODULE_2.position,
      MODULE_3.position);

  public static final class constControllers {
    public static final double DRIVER_LEFT_STICK_X_DEADBAND = 0.1;
    public static final double DRIVER_LEFT_STICK_Y_DEADBAND = 0.1;
    public static final double DRIVER_RIGHT_STICK_X_DEADBAND = 0.1;
    public static final double DRIVER_RIGHT_STICK_Y_DEADBAND = 0.1;
    public static final double DRIVER_LEFT_TRIGGER_DEADBAND = 0.0;
    public static final double DRIVER_RIGHT_TRIGGER_DEADBAND = 0.0;

    public static final double OPERATOR_LEFT_STICK_X_DEADBAND = 0.1;
    public static final double OPERATOR_LEFT_STICK_Y_DEADBAND = 0.1;
    public static final double OPERATOR_RIGHT_STICK_X_DEADBAND = 0.1;
    public static final double OPERATOR_RIGHT_STICK_Y_DEADBAND = 0.1;
    public static final double OPERATOR_LEFT_TRIGGER_DEADBAND = 0.0;
    public static final double OPERATOR_RIGHT_TRIGGER_DEADBAND = 0.0;
  }

  public static final class constChargerTreads {
    public static final boolean LEFT_MOTOR_INVERTED = false;
    public static final boolean RIGHT_MOTOR_INVERTED = true;

    public static final int RESET_ENCODERS = 0;
  }

  public static final class constIntake {
    public static final boolean LEFT_MOTOR_INVERTED = false;
    public static final boolean RIGHT_MOTOR_INVERTED = true;

    // RGB game piece colors
    public static final double coneColorR = 0.34509;
    public static final double coneColorG = 0.51764;
    public static final double coneColorB = 0.13333;

    public static final double cubeColorR = 0.22745;
    public static final double cubeColorG = 0.39607;
    public static final double cubeColorB = 0.37254;
  }

  public static final class constCollector {
    public static final double GEAR_RATIO = 100;

    public static final boolean PIVOT_FORWARD_LIMIT_ENABLE = true;
    public static final boolean PIVOT_REVERSE_LIMIT_ENABLE = true;

    public static final double PIVOT_FORWARD_LIMIT_VALUE = Units.degreesToRadians(100);
    public static final double PIVOT_REVERSE_LIMIT_VALUE = Units.degreesToRadians(0);
  }

  public static final class constArm {
    public static final boolean SHOULDER_MOTOR_INVERT = false;
    public static final boolean ELBOW_MOTOR_INVERT = false;

    public static final NeutralMode SHOULDER_MOTOR_BREAK = NeutralMode.Brake;
    public static final NeutralMode ELBOW_MOTOR_BREAK = NeutralMode.Brake;

    public static final double SHOULDER_GEAR_RATIO = 100.0;
    public static final double ELBOW_GEAR_RATIO = 100.0;

    // offsets are when both joints are facing to the right (0 degrees on unit
    // circle is at (1, 0))
    public static final double SHOULDER_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.397309);
    public static final double ELBOW_ABSOLUTE_ENCODER_OFFSET = Units.rotationsToRadians(0.142530);

    public static final double SHOULDER_LENGTH = Units.inchesToMeters(7.5);
    public static final double ELBOW_LENGTH = Units.inchesToMeters(8.75);

    public static final double SHOULDER_FORWARD_LIMIT = Units.degreesToRadians(180);
    public static final double SHOULDER_REVERSE_LIMIT = Units.degreesToRadians(-90);

    public static final double ELBOW_FORWARD_LIMIT = Units.degreesToRadians(163);
    public static final double ELBOW_REVERSE_LIMIT = Units.degreesToRadians(-170);
  }

}
