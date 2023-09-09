// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor;

  DigitalInput limitSwitch;

  TalonFXConfiguration config;
  StatorCurrentLimitConfiguration currentLimitConfig;

  public Intake() {
    intakeMotor = new TalonFX(mapIntake.INTAKE_OUTSIDE_MOTOR_CAN);
    limitSwitch = new DigitalInput(mapIntake.INTAKE_LIMIT_SWITCH_DIO);

    config = new TalonFXConfiguration();

    configure();
  }

  public void configure() {
    intakeMotor.configFactoryDefault();
    intakeMotor.configAllSettings(config);

    // https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html?highlight=Current%20limit#new-api-in-2020
    currentLimitConfig = new StatorCurrentLimitConfiguration(true, constIntake.CURRENT_LIMIT_TO_AMPS,
        constIntake.CURRENT_LIMIT_AT_AMPS, constIntake.CURRENT_LIMIT_AFTER_MS);

    intakeMotor.configStatorCurrentLimit(currentLimitConfig);
  }

  // will always spin opposite the inside motor
  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean isGamePieceCollected() {
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
