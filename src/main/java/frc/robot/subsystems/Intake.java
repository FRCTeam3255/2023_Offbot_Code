// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor;

  TalonFXConfiguration config;
  SupplyCurrentLimitConfiguration currentLimitConfig;

  private boolean isGamePieceCollected;
  GamePiece currentGamePiece;

  public Intake() {
    intakeMotor = new TalonFX(mapIntake.INTAKE_OUTSIDE_MOTOR_CAN);
    config = new TalonFXConfiguration();

    configure();
  }

  public void configure() {
    intakeMotor.configFactoryDefault();
    intakeMotor.configAllSettings(config);

    // https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html?highlight=Current%20limit#new-api-in-2020
    currentLimitConfig = new SupplyCurrentLimitConfiguration(true, constIntake.CURRENT_LIMIT_TO_AMPS,
        constIntake.CURRENT_LIMIT_AT_AMPS, constIntake.CURRENT_LIMIT_AFTER_MS);

    intakeMotor.configSupplyCurrentLimit(currentLimitConfig);
  }

  /**
   * Set the speed of the rollers.
   * 
   * @param speed Desired speed to set the motor to, as a PercentOutput
   *              (-1.0 to 1.0)
   * 
   */
  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Uses the intake motor's draw (input bus voltage) to determine if a game piece
   * is collected. Should be run periodically.
   */
  private void setGamePieceCollected() {
    if (intakeMotor.getSupplyCurrent() > constIntake.CURRENT_LIMIT_AT_AMPS) {
      isGamePieceCollected = true;
      return;
    }
    isGamePieceCollected = false;
  }

  /**
   * Used to set the current game piece collected. Will check if we have a
   * gamepiece before setting the gamepiece.
   * 
   * @param gamepiece Desired collected game piece
   */
  public void setCurrentGamePiece(GamePiece gamepiece) {
    if (isGamePieceCollected) {
      currentGamePiece = gamepiece;
      return;
    }
    currentGamePiece = GamePiece.NONE;
  }

  /**
   * Return if the current game piece is collected
   * 
   * @return If the current game piece is collected
   */
  public boolean isGamePieceCollected() {
    return isGamePieceCollected;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setGamePieceCollected();
  }
}
