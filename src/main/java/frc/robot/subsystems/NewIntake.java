// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapIntake;

public class NewIntake extends SubsystemBase {
  /** Creates a new NewIntake. */
  TalonFX outsideMotor;
  TalonFX insideMotor;

  public NewIntake() {
    outsideMotor = new TalonFX(mapIntake.INTAKE_OUTSIDE_MOTOR_CAN);
    insideMotor = new TalonFX(mapIntake.INTAKE_INSIDE_MOTOR_CAN);

    configure();
  }

  public void configure() {
    outsideMotor.configFactoryDefault();
    insideMotor.configFactoryDefault();
  }

  public void setOutsideMotorSpeed(double speed) {
    outsideMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setInsideMotorSpeed(double speed) {
    insideMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
