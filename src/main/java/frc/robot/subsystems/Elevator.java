// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DesiredHeight;
import frc.robot.RobotMap.mapElevator;
import frc.robot.RobotPreferences.prefElevator;

public class Elevator extends SubsystemBase {

  TalonFX leftMotor;
  TalonFX rightMotor;

  TalonFXConfiguration config;

  DesiredHeight desiredHeight;
  double desiredPosition;

  public Elevator() {
    leftMotor = new TalonFX(mapElevator.LEFT_MOTOR_CAN);
    rightMotor = new TalonFX(mapElevator.RIGHT_MOTOR_CAN);
    config = new TalonFXConfiguration();

    desiredHeight = DesiredHeight.NONE;

    configure();
  }

  public void configure() {

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder Counts", getElevatorEncoderCounts());
    SmartDashboard.putNumber("Elevator Distance Feet", getElevatorPositionFeet());
  }
}
