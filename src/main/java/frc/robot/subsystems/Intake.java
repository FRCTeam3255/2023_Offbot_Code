// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;
import frc.robot.RobotPreferences.prefIntake;

public class Intake extends SubsystemBase {
  TalonFX intakeMotor;
  DigitalInput intakeLimitSwitch;

  TalonFXConfiguration config;
  StatorCurrentLimitConfiguration statorLimit;

  GamePiece currentGamePiece = GamePiece.NONE;
  GamePiece desiredGamePiece = GamePiece.NONE;

  double intakeTimer = 0;

  public Intake() {
    intakeMotor = new TalonFX(mapIntake.INTAKE_MOTOR_CAN);
    intakeLimitSwitch = new DigitalInput(mapIntake.INTAKE_LIMIT_SWITCH_DIO);

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

  public boolean getIntakeLimitSwitch() {
    return false; // Disabling the limit switch
    // return prefIntake.intakeLimitSwitchInvert.getValue() ?
    // !intakeLimitSwitch.get() : intakeLimitSwitch.get();
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

  public void setCurrentLimiting(boolean status) {
    // https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html?highlight=Current%20limit#new-api-in-2020
    intakeMotor
        .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(status, constIntake.CURRENT_LIMIT_FLOOR_AMPS,
            constIntake.CURRENT_LIMIT_CEILING_AMPS, constIntake.CURRENT_LIMIT_AFTER_SEC));
  }

  // --- Game Piece Logic Start ---

  /**
   * Set what game piece we would like to have.
   * 
   * @param gamePiece What game piece we would like to have
   */
  public void setDesiredGamePiece(GamePiece gamePiece) {
    this.desiredGamePiece = gamePiece;
  }

  /**
   * Uses the intake motor's draw (input bus voltage) and/or the limit switch to
   * determine if a game piece is collected.
   * 
   * @return If a game piece is collected
   */
  public boolean isGamePieceCollected() {
    double current = intakeMotor.getStatorCurrent();
    boolean hasGamePiece = (current < prefIntake.intakePieceCollectedBelowAmps.getValue()
        && current > prefIntake.intakePieceCollectedAboveAmps.getValue()) || getIntakeLimitSwitch();

    if (hasGamePiece && intakeTimer <= prefIntake.intakePieceCollectedDebounce.getValue()) {
      intakeTimer += 1;
    } else if (!hasGamePiece) {
      intakeTimer = 0;
    } else {
      return true;
    }
    return false;
  }

  /**
   * Sets the current game piece to the desired game piece if we have a game
   * piece. Should be run periodically.
   */
  private void setCurrentGamePiece() {
    if (isGamePieceCollected()) {
      currentGamePiece = desiredGamePiece;
      return;
    }
    currentGamePiece = GamePiece.NONE;
  }

  /**
   * @return the current game piece collected
   */
  public GamePiece getCurrentGamePiece() {
    return currentGamePiece;
  }

  /**
   * @return the desired game piece
   */
  public GamePiece getDesiredGamePiece() {
    return desiredGamePiece;
  }

  // --- Game Piece Logic End ---

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setCurrentGamePiece();
    SmartDashboard.putNumber("Intake Game Piece Timer", intakeTimer);

    SmartDashboard.putBoolean("Is Game Piece Collected", isGamePieceCollected());
    SmartDashboard.putString("Current Game Piece", getCurrentGamePiece().toString());
    SmartDashboard.putString("Desired Game Piece", getDesiredGamePiece().toString());
    SmartDashboard.putNumber("Intake STATOR AMPS", intakeMotor.getStatorCurrent());
  }
}
