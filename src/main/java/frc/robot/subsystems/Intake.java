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
  boolean isCurrentLimitingOn = true;

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

  // public boolean isIntakeAtVelocity(double velocity) {
  // return Math.abs(intakeMotor.getSelectedSensorVelocity() - velocity) < 1;
  // }

  public void setCurrentLimiting(boolean status) {
    // https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html?highlight=Current%20limit#new-api-in-2020
    intakeMotor
        .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(status, constIntake.CURRENT_LIMIT_FLOOR_AMPS,
            constIntake.CURRENT_LIMIT_CEILING_AMPS, constIntake.CURRENT_LIMIT_AFTER_SEC));
    isCurrentLimitingOn = status;
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

  boolean isAtVelocity = false;
  boolean test = false;

  /**
   * Uses the intake motor's draw (input bus voltage) and/or the limit switch to
   * determine if a game piece is collected.
   * 
   * @return If a game piece is collected
   */
  public boolean isGamePieceCollected() {
    double current = intakeMotor.getStatorCurrent();
    double desiredVelocity = prefIntake.intakeConeVelocityTolerance.getValue();
    double belowCurrent = prefIntake.intakePieceConeCollectedBelowAmps.getValue();
    double aboveCurrent = prefIntake.intakePieceConeCollectedAboveAmps.getValue();

    if (getDesiredGamePiece().equals(GamePiece.CUBE)) {
      desiredVelocity = prefIntake.intakeCubeVelocityTolerance.getValue();
      belowCurrent = prefIntake.intakePieceCubeCollectedBelowAmps.getValue();
      aboveCurrent = prefIntake.intakePieceCubeCollectedAboveAmps.getValue();
    }

    // TODO: REMOVE
    isAtVelocity = Math.abs(intakeMotor.getSelectedSensorVelocity()) < Math
        .abs(desiredVelocity);

    test = current < belowCurrent && current > aboveCurrent;

    if (current < belowCurrent
        && current > aboveCurrent
        && Math.abs(intakeMotor.getSelectedSensorVelocity()) < Math
            .abs(desiredVelocity)) {
      return true;
    } else {
      return false;
    }
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

    // TODO: REMOVE THESE
    SmartDashboard.putBoolean("Intake bool velocity",
        isAtVelocity);
    SmartDashboard.putBoolean("Intake bool current",
        test);

    SmartDashboard.putNumber("Intake STATOR AMPS", intakeMotor.getStatorCurrent());
    SmartDashboard.putNumber("Intake Velocity", intakeMotor.getSelectedSensorVelocity());

  }
}
