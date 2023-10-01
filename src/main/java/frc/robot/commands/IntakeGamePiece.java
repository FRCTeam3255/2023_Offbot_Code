// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constLEDs;
import frc.robot.Constants.GamePiece;
import frc.robot.RobotPreferences.prefElevator;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class IntakeGamePiece extends SequentialCommandGroup {
  Wrist subWrist;
  Intake subIntake;
  Elevator subElevator;
  LEDs subLEDs;
  GamePiece gamepiece;

  double intakeSpeed;
  PatternType pattern;

  double wristPosition;
  double elevatorPositon;

  public IntakeGamePiece(Wrist subWrist, Intake subIntake, Elevator subElevator, GamePiece gamepiece,
      double wristPosition, double elevatorPosition) {

    this.subWrist = subWrist;
    this.subIntake = subIntake;
    this.subElevator = subElevator;
    this.gamepiece = gamepiece;
    this.wristPosition = wristPosition;
    this.elevatorPositon = elevatorPosition;

    addRequirements(subWrist, subIntake, subElevator);

    // Assume its a cone if there is no value (fallback condition, should never
    // happen)
    if (gamepiece == GamePiece.CUBE) {
      pattern = constLEDs.INTAKING_CUBE_COLOR;
    } else {
      pattern = constLEDs.INTAKING_CONE_COLOR;
    }

    addCommands(
        Commands.parallel(
            Commands.runOnce(() -> subElevator.setElevatorPosition(elevatorPosition))),

        // Commands.runOnce(() -> subLEDs.setLEDPattern(pattern))),

        // Commands.waitUntil(() ->
        // subElevator.isElevatorAtPosition(prefElevator.elevatorIntakingPos.getValue())
        // == true),

        Commands.runOnce(() -> subWrist.setWristAngle(wristPosition)),

        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(prefIntake.intakeCubeSpeed.getValue()))
            .unless(() -> !gamepiece.equals(GamePiece.CUBE)),
        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(prefIntake.intakeConeSpeed.getValue()))
            .unless(() -> !gamepiece.equals(GamePiece.CONE)),

        Commands.waitUntil(() -> subIntake.isGamePieceCollected()),

        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subIntake.setCurrentGamePiece(gamepiece)),
        Commands.runOnce(() -> subElevator.setElevatorPosition(prefElevator.elevatorStow.getValue())));

  }
}
