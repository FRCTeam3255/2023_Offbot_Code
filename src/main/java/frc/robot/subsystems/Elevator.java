// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.DesiredHeight;
import frc.robot.RobotMap.mapElevator;
import frc.robot.RobotPreferences.prefElevator;

public class Elevator extends SubsystemBase {

  TalonFX leftMotor;
  TalonFX rightMotor;

  DutyCycleEncoder absoluteEncoder;
  double absoluteEncoderOffset;

  TalonFXConfiguration config;
  StatorCurrentLimitConfiguration statorLimit;

  DesiredHeight desiredHeight;
  double desiredPosition;
  boolean isPrepped;

  public Elevator() {
    leftMotor = new TalonFX(mapElevator.LEFT_MOTOR_CAN);
    rightMotor = new TalonFX(mapElevator.RIGHT_MOTOR_CAN);
    config = new TalonFXConfiguration();

    absoluteEncoder = new DutyCycleEncoder(mapElevator.ELEVATOR_ABSOLUTE_ENCODER_DIO);
    absoluteEncoderOffset = constElevator.ABSOLUTE_ENCODER_OFFSET;

    desiredHeight = DesiredHeight.NONE;

    configure();
  }

  public void configure() {
    if (absoluteEncoder.getAbsolutePosition() > constElevator.ABSOLUTE_ENCODER_ROLLOVER_OFFSET) {
      absoluteEncoder.setPositionOffset(1);
    }

    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.configAllSettings(config);
    rightMotor.configAllSettings(config);

    config.slot0.kF = prefElevator.elevatorF.getValue();
    config.slot0.kP = prefElevator.elevatorP.getValue();
    config.slot0.kI = prefElevator.elevatorI.getValue();
    config.slot0.kD = prefElevator.elevatorD.getValue();

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    // https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html?highlight=Current%20limit#new-api-in-2020
    statorLimit = new StatorCurrentLimitConfiguration(true, constElevator.CURRENT_LIMIT_FLOOR_AMPS,
        constElevator.CURRENT_LIMIT_CEILING_AMPS, constElevator.CURRENT_LIMIT_AFTER_SEC);

    rightMotor.configStatorCurrentLimit(statorLimit);
    leftMotor.configStatorCurrentLimit(statorLimit);

    leftMotor.follow(rightMotor);
  }

  /**
   * Set the speed of the Elevator. Includes safeties/soft stops.
   * 
   * @param speed Desired speed to set both of the motors to, as a PercentOutput
   *              (-1.0 to 1.0)
   * 
   */
  public void setElevatorSpeed(double speed) {
    // don't go past the max position
    if (getElevatorPositionFeet() > prefElevator.elevatorMaxPos.getValue() && speed > 0) {
      speed = 0;
    }

    // don't go past the min position
    if (getElevatorPositionFeet() < prefElevator.elevatorMinPos.getValue() && speed < 0) {
      speed = 0;
    }

    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set the position of the Elevator. Includes safeties/soft stops.
   * 
   * @param position Desired position to set both of the motors to, in Encoder
   *                 ticks
   * 
   */
  public void setElevatorPosition(double position) {
    position = MathUtil.clamp(position, prefElevator.elevatorMinPos.getValue(),
        prefElevator.elevatorMaxPos.getValue());

    leftMotor.set(ControlMode.Position, position);
    rightMotor.set(ControlMode.Position, position);
  }

  /**
   * Returns if the elevator is within the tolerance of a given position.
   * 
   * @param position The position (in encoder ticks) to check
   * @return If it is at that position
   * 
   */
  public boolean isElevatorAtPosition(double position) {
    position = MathUtil.clamp(position, prefElevator.elevatorMinPos.getValue(),
        prefElevator.elevatorMaxPos.getValue());

    return prefElevator.elevatorPositionTolerance.getValue() > Math.abs(getElevatorEncoderCounts() - position);
  }

  /**
   * Returns the encoder counts of one motor on the elevator. (They should have
   * the same reading)
   * 
   * @return Elevator encoder counts
   * 
   */
  public double getElevatorEncoderCounts() {
    return rightMotor.getSelectedSensorPosition();
  }

  private double getElevatorAbsoluteEncoder() {
    double rotations = absoluteEncoder.get();
    rotations -= absoluteEncoderOffset;

    if (constElevator.ABSOLUTE_ENCODER_INVERT) {
      return -rotations;
    } else {
      return rotations;
    }
  }

  public void resetElevatorEncoderToAbsolute() {
    rightMotor.setSelectedSensorPosition(
        SN_Math.degreesToFalcon(Units.rotationsToDegrees(getElevatorAbsoluteEncoder()),
            constElevator.GEAR_RATIO));
  }

  /**
   * Returns the position of the elevator, relative to itself, in feet.
   * 
   * @return Elevator position
   * 
   */
  public double getElevatorPositionFeet() {
    return getElevatorEncoderCounts() / prefElevator.elevatorEncoderCountsPerFoot.getValue();
  }

  public void setDesiredHeight(DesiredHeight height) {
    height = desiredHeight;
  }

  public DesiredHeight getDesiredHeight() {
    return desiredHeight;
  }

  public void setIsPrepped(boolean prepped) {
    prepped = isPrepped;
  }

  public boolean isPrepped() {
    return isPrepped;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder Counts Raw", getElevatorEncoderCounts());
    SmartDashboard.putNumber("Elevator Distance Feet", getElevatorPositionFeet());

    SmartDashboard.putNumber("Elevator Abs Encoder Raw", absoluteEncoder.get());
    SmartDashboard.putNumber("Elevator Abs Encoder Abs", absoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Elevator Abs Encoder Get", getElevatorAbsoluteEncoder());
  }
}
