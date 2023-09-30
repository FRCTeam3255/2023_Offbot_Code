// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor;

  TalonFXConfiguration config;
  StatorCurrentLimitConfiguration statorLimit;

  boolean isGamePieceCollected;
  GamePiece currentGamePiece = GamePiece.NONE;

  public Intake() {
    intakeMotor = new TalonFX(mapIntake.INTAKE_MOTOR_CAN);
    config = new TalonFXConfiguration();

    configure();
  }

  public void configure() {
    intakeMotor.configFactoryDefault();
    intakeMotor.configAllSettings(config);
    intakeMotor.setInverted(constIntake.MOTOR_INVERTED);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    // https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html?highlight=Current%20limit#new-api-in-2020
    statorLimit = new StatorCurrentLimitConfiguration(true, constIntake.CURRENT_LIMIT_FLOOR_AMPS,
        constIntake.CURRENT_LIMIT_CEILING_AMPS, constIntake.CURRENT_LIMIT_AFTER_SEC);

    intakeMotor.configStatorCurrentLimit(statorLimit);
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
    double current = intakeMotor.getStatorCurrent();
    if (current > 10) {
      isGamePieceCollected = true;
    } else if (current < 9) {
      isGamePieceCollected = false;
    }
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

  public GamePiece getCurrentGamePiece() {
    return currentGamePiece;
  }

  public void setCurrentLimiting(boolean status) {
    // https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html?highlight=Current%20limit#new-api-in-2020

    intakeMotor
        .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(status, constIntake.CURRENT_LIMIT_FLOOR_AMPS,
            constIntake.CURRENT_LIMIT_CEILING_AMPS, constIntake.CURRENT_LIMIT_AFTER_SEC));
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
    SmartDashboard.putBoolean("Is Game Piece Collected", isGamePieceCollected());
    SmartDashboard.putString("game piece", getCurrentGamePiece().toString());
    SmartDashboard.putNumber("Intake STATOR AMPS", intakeMotor.getStatorCurrent());
  }
}
