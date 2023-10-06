// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DesiredHeight;
import frc.robot.Constants.GamePiece;
import frc.robot.RobotPreferences.prefElevator;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGamePiece extends SequentialCommandGroup {
  Intake subIntake;
  Wrist subWrist;
  Elevator subElevator;

  double speed;

  public PlaceGamePiece(Intake subIntake, Wrist subWrist, Elevator subElevator) {
    this.subIntake = subIntake;
    this.subWrist = subWrist;
    this.subElevator = subElevator;

    addCommands(
        Commands.runOnce(() -> subIntake.setCurrentLimiting(false)),

        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(prefIntake.intakePlaceCubeSpeed.getValue()))
            .unless(() -> !subIntake.getDesiredGamePiece().equals(GamePiece.CUBE)),
        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(prefIntake.intakePlaceConeSpeed.getValue()))
            .unless(() -> subIntake.getDesiredGamePiece().equals(GamePiece.CUBE)),

        Commands.waitUntil(() -> !subIntake.isGamePieceCollected()),
        Commands.waitSeconds(prefIntake.intakeMidConeDelay.getValue())
            .unless(() -> !subElevator.getDesiredHeight().equals(DesiredHeight.MID)
                || !subIntake.getDesiredGamePiece().equals(GamePiece.CONE)),

        Commands.runOnce(() -> subIntake.setDesiredGamePiece(GamePiece.NONE)),

        Commands.runOnce(
            () -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),
        Commands.waitUntil(() -> subWrist.isWristAtPosition(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subElevator.setElevatorPosition(prefElevator.elevatorStow.getValue())),

        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(0)),

        Commands.runOnce(() -> subIntake.setCurrentLimiting(true)),
        Commands.runOnce(() -> subElevator.setIsPrepped(false)),

        Commands.waitUntil(() -> subElevator.isElevatorAtPosition(prefElevator.elevatorStow.getValue(), 0.1)),
        Commands.runOnce(() -> subElevator.neutralElevatorOutputs()));

  }
}
