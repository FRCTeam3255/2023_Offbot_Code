// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constWrist;
import frc.robot.RobotMap.mapWrist;
import frc.robot.RobotPreferences.prefWrist;

public class Wrist extends SubsystemBase {

  TalonFX wristMotor;

  DutyCycleEncoder absoluteEncoder;
  double absoluteEncoderOffset;

  TalonFXConfiguration config;

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
    }

    wristMotor.configFactoryDefault();

    wristMotor.setNeutralMode(NeutralMode.Brake);

    config.forwardSoftLimitThreshold = SN_Math.degreesToFalcon(prefWrist.wristMaxPos.getValue(),
        constWrist.GEAR_RATIO);
    config.reverseSoftLimitThreshold = SN_Math.degreesToFalcon(prefWrist.wristMinPos.getValue(),
        constWrist.GEAR_RATIO);

    config.forwardSoftLimitEnable = true;
    config.reverseSoftLimitEnable = true;

    config.slot0.allowableClosedloopError = SN_Math.degreesToFalcon(prefWrist.wristPositionTolerance.getValue(),
        constWrist.GEAR_RATIO);
    config.motionCruiseVelocity = SN_Math.degreesToFalcon(prefWrist.wristMaxVelocity.getValue(),
        constWrist.GEAR_RATIO);
    config.motionAcceleration = SN_Math.degreesToFalcon(prefWrist.wristMaxAccel.getValue(),
        constWrist.GEAR_RATIO);

    config.slot0.kF = prefWrist.wristF.getValue();
    config.slot0.kP = prefWrist.wristP.getValue();
    config.slot0.kI = prefWrist.wristI.getValue();
    config.slot0.kD = prefWrist.wristD.getValue();

    wristMotor.configAllSettings(config);
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

    wristMotor.set(ControlMode.MotionMagic, SN_Math.degreesToFalcon(angle, constWrist.GEAR_RATIO));
  }

  /**
   * @return The angle of the wrist motor, as a Rotation2d
   */
  public Rotation2d getWristAngle() {
    return Rotation2d
        .fromDegrees(SN_Math.falconToDegrees(wristMotor.getSelectedSensorPosition(), constWrist.GEAR_RATIO));
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
   * Reset the wrist motor to the offset value of the absolute encoder.
   */
  public void resetWristEncoderToAbsolute() {
    wristMotor.setSelectedSensorPosition(
        SN_Math.degreesToFalcon(Units.rotationsToDegrees(getWristAbsoluteEncoder()),
            constWrist.GEAR_RATIO));
  }

  public void neutralElevatorOutputs() {
    wristMotor.neutralOutput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Abs Encoder Raw", absoluteEncoder.get());
    SmartDashboard.putNumber("Abs Encoder Abs", absoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Abs Encoder Get", getWristAbsoluteEncoder());

    SmartDashboard.putNumber("Wrist Motor Degrees", getWristAngle().getDegrees());

  }
}
