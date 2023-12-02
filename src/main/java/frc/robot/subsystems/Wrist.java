// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AdvantageScopeUtil;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.constWrist;
import frc.robot.RobotMap.mapWrist;
import frc.robot.RobotPreferences.prefWrist;

public class Wrist extends SubsystemBase {

  TalonFX wristMotor;

  DutyCycleEncoder absoluteEncoder;
  double absoluteEncoderOffset;

  TalonFXConfiguration config;

  SupplyCurrentLimitConfiguration supplyLimit;

  Pose3d desiredWristPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  Pose3d wristPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  double desiredAngle = 0;

  public Wrist() {
    wristMotor = new TalonFX(mapWrist.WRIST_MOTOR_CAN);
    config = new TalonFXConfiguration();

    absoluteEncoder = new DutyCycleEncoder(mapWrist.WRIST_ABSOLUTE_ENCODER_DIO);
    absoluteEncoderOffset = constWrist.ABSOLUTE_ENCODER_OFFSET;

    configure();
  }

  public void configure() {
    if (absoluteEncoder.getAbsolutePosition() > constWrist.ABSOLUTE_ENCODER_ROLLOVER_OFFSET) {
      absoluteEncoder.setPositionOffset(1);
    } else {
      absoluteEncoder.setPositionOffset(0);
    }

    wristMotor.configFactoryDefault();

    wristMotor.setNeutralMode(NeutralMode.Brake);

    config.forwardSoftLimitThreshold = SN_Math.degreesToFalcon(prefWrist.wristMaxPos.getValue(),
        constWrist.GEAR_RATIO);
    config.reverseSoftLimitThreshold = SN_Math.degreesToFalcon(prefWrist.wristMinPos.getValue(),
        constWrist.GEAR_RATIO);

    config.forwardSoftLimitEnable = true;
    config.reverseSoftLimitEnable = true;

    config.slot0.allowableClosedloopError = SN_Math.degreesToFalcon(prefWrist.wristPIDTolerance.getValue(),
        constWrist.GEAR_RATIO);
    config.motionCruiseVelocity = SN_Math.degreesToFalcon(prefWrist.wristMaxVelocity.getValue(),
        constWrist.GEAR_RATIO);
    config.motionAcceleration = SN_Math.degreesToFalcon(prefWrist.wristMaxAccel.getValue(),
        constWrist.GEAR_RATIO);

    // https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html?highlight=Current%20limit#new-api-in-2020
    supplyLimit = new SupplyCurrentLimitConfiguration(true,
        constWrist.CURRENT_LIMIT_FLOOR_AMPS,
        constWrist.CURRENT_LIMIT_CEILING_AMPS,
        constWrist.CURRENT_LIMIT_AFTER_SEC);

    config.slot0.kF = prefWrist.wristF.getValue();
    config.slot0.kP = prefWrist.wristP.getValue();
    config.slot0.kI = prefWrist.wristI.getValue();
    config.slot0.kD = prefWrist.wristD.getValue();

    wristMotor.configAllSettings(config);

    wristMotor.configSupplyCurrentLimit(supplyLimit);
  }

  public void setWristSpeed(double speed) {
    wristMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set the angle of the wrist. Includes safeties/soft stops.
   * 
   * @param angle Desired angle to set the motor to, in Encoder
   *              ticks
   * 
   */
  public void setWristAngle(double angle) {
    angle = MathUtil.clamp(angle, prefWrist.wristMinPos.getValue(),
        prefWrist.wristMaxPos.getValue());
    desiredAngle = angle;

    wristMotor.set(ControlMode.MotionMagic, SN_Math.degreesToFalcon(angle, constWrist.GEAR_RATIO));
  }

  public boolean isWristAtAngle(double angle) {
    return SN_Math.degreesToFalcon(prefWrist.wristAngleTolerance.getValue(), constWrist.GEAR_RATIO) >= Math
        .abs(wristMotor.getClosedLoopError());
  }

  /**
   * @return The real angle of the wrist motor, as a Rotation2d
   */
  public Rotation2d getWristAngle() {
    return Rotation2d
        .fromDegrees(SN_Math.falconToDegrees(wristMotor.getSelectedSensorPosition(), constWrist.GEAR_RATIO));
  }

  /**
   * @return The desired angle of the wrist motor, as a Rotation2d
   */
  public Rotation2d getDesiredWristAngle() {
    return Rotation2d.fromDegrees(desiredAngle);
  }

  /**
   * Returns if the wrist is within its positional tolerance.
   * 
   * @param desiredPosition Desired position, in degrees
   * @return If it is at that position
   * 
   */
  public boolean isWristAtPosition(double desiredPosition) {
    if (Robot.isSimulation()) {
      return true;
    }
    return prefWrist.wristPositionTolerance.getValue() >= Math.abs(getWristAngle().getDegrees() - desiredPosition);
  }

  /**
   * Get the Wrist absolute encoder reading with the offset applied.
   * 
   * @return Wrist absolute encoder reading in rotations
   */
  private double getWristAbsoluteEncoder() {
    double rotations = absoluteEncoder.get();
    rotations -= absoluteEncoderOffset;

    if (constWrist.ABSOLUTE_ENCODER_INVERT) {
      return -rotations;
    } else {
      return rotations;
    }
  }

  /**
   * Gets if the absolute encoder was plugged in on init. Returns true if it was
   * unplugged. This will also return true in simulation, because there is no
   * absolute encoder to get a value from.
   */
  public boolean getWristEncoderUnplugged() {
    return absoluteEncoder.get() == 0.0;
  }

  /**
   * Reset the wrist motor to the offset value of the absolute encoder.
   */
  public void resetWristEncoderToAbsolute() {
    if (getWristEncoderUnplugged()) {
      // todo: add default value to reset wrist to
      return;
    }

    wristMotor.setSelectedSensorPosition(
        SN_Math.degreesToFalcon(Units.rotationsToDegrees(getWristAbsoluteEncoder()),
            constWrist.GEAR_RATIO));
  }

  public void neutralElevatorOutputs() {
    wristMotor.neutralOutput();
  }

  // Must be run every loop in order for logging to function
  public void updatePose3ds(Pose3d elevatorCarriagePose, Pose3d desiredElevatorCarriagePose) {
    wristPose = elevatorCarriagePose.transformBy(new Transform3d(new Pose3d(),
        new Pose3d(-0.298, 0.005, 0.218, new Rotation3d(0, -getWristAngle().getRadians(), 0))));
    desiredWristPose = desiredElevatorCarriagePose.transformBy(new Transform3d(new Pose3d(),
        new Pose3d(-0.298, 0.005, 0.218, new Rotation3d(0, -getDesiredWristAngle().getRadians(), 0))));
  }

  @Override
  public void periodic() {
    updatePose3ds(RobotContainer.subElevator.getElevatorCarriagePose(),
        RobotContainer.subElevator.getDesiredElevatorCarriagePose());

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist/Abs Encoder Raw", absoluteEncoder.get());
    SmartDashboard.putNumber("Wrist/Abs Encoder Abs", absoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Wrist/Abs Encoder Get", getWristAbsoluteEncoder());
    SmartDashboard.putBoolean("Wrist/Abs Encoder Unplugged?", getWristEncoderUnplugged());
    SmartDashboard.putNumber("Wrist/Motor Degrees", getWristAngle().getDegrees());
    SmartDashboard.putNumber("Wrist/Supply Current", wristMotor.getSupplyCurrent());
    SmartDashboard.putNumberArray("Wrist/Pose3d", AdvantageScopeUtil.composePose3ds(wristPose));
    SmartDashboard.putNumberArray("Wrist/Desired Pose3d", AdvantageScopeUtil.composePose3ds(desiredWristPose));
  }
}
