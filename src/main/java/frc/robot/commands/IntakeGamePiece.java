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
  double elevatorPosition;

  public IntakeGamePiece(Wrist subWrist, Intake subIntake, Elevator subElevator, LEDs subLEDs, GamePiece gamepiece,
      double wristPosition, double elevatorPosition) {

    this.subWrist = subWrist;
    this.subIntake = subIntake;
    this.subElevator = subElevator;
    this.subLEDs = subLEDs;
    this.gamepiece = gamepiece;
    this.wristPosition = wristPosition;
    this.elevatorPosition = elevatorPosition;

    addRequirements(subWrist, subIntake, subElevator, subLEDs);

    addCommands(
        Commands.runOnce(() -> subIntake.setDesiredGamePiece(gamepiece)),
        Commands.runOnce(() -> subIntake.setCurrentLimiting(false)),
        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(prefIntake.intakeCubeSpeed.getValue()))
            .alongWith(Commands.runOnce(() -> subLEDs.setLEDsToAnimation(constLEDs.INTAKING_CUBE_COLOR)))
            .unless(() -> !gamepiece.equals(GamePiece.CUBE)),
        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(prefIntake.intakeConeSpeed.getValue()))
            .alongWith(Commands.runOnce(() -> subLEDs.setLEDsToAnimation(constLEDs.INTAKING_CONE_COLOR)))
            .unless(() -> gamepiece.equals(GamePiece.CUBE)),
        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subElevator.setElevatorPosition(elevatorPosition)),
        Commands.waitUntil(() -> subElevator.isElevatorAtPosition(elevatorPosition,
            prefElevator.elevatorWristPositionTolerance.getValue())),

        Commands.runOnce(() -> subWrist.setWristAngle(wristPosition)),
        Commands.waitUntil(() -> subWrist.isWristAtPosition(wristPosition)),
        Commands.runOnce(() -> subIntake.setCurrentLimiting(true)),

        Commands.waitUntil(() -> subIntake.isGamePieceCollected()),

        Commands.runOnce(() -> subLEDs.clearAnimation()),
        Commands.runOnce(() -> subLEDs.setLEDs(constLEDs.HAS_CUBE_COLOR))
            .unless(() -> !gamepiece.equals(GamePiece.CUBE)),
        Commands.runOnce(() -> subLEDs.setLEDs(constLEDs.HAS_CONE_COLOR))
            .unless(() -> gamepiece.equals(GamePiece.CUBE)),

        Commands.waitSeconds(prefIntake.intakeDelay.getValue()),

        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),
        Commands.waitUntil(() -> subWrist.isWristAtPosition(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subElevator.setElevatorPosition(prefElevator.elevatorStow.getValue())),
        Commands.waitUntil(() -> subElevator.isElevatorAtPosition(prefElevator.elevatorStow.getValue(),
            prefElevator.elevatorActualPositionTolerance.getValue())),
        Commands.runOnce(() -> subElevator.neutralElevatorOutputs()));

  }
}
