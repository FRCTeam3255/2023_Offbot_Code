// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GamePiece;
import frc.robot.RobotPreferences.prefElevator;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class IntakeGamePiece extends SequentialCommandGroup {
  Wrist subWrist;
  Intake subIntake;
  Elevator subElevator;
  GamePiece gamepiece;

  double intakeSpeed;

  public IntakeGamePiece(Wrist subWrist, Intake subIntake, Elevator subElevator, GamePiece gamepiece) {

    this.subWrist = subWrist;
    this.subIntake = subIntake;
    this.subElevator = subElevator;
    this.gamepiece = gamepiece;

    // Assume its a cone if there is no value (fallback condition, should never
    // happen)
    if (gamepiece == GamePiece.CUBE) {
      intakeSpeed = prefIntake.intakeCubeSpeed.getValue();
    } else {
      intakeSpeed = prefIntake.intakeConeSpeed.getValue();
    }

    addCommands(

        Commands.runOnce(() -> subElevator.setElevatorPosition(prefElevator.elevatorIntakingPos.getValue())),

        Commands.waitUntil(() -> subElevator.isElevatorAtPosition(prefElevator.elevatorIntakingPos.getValue()) == true),

        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristIntakingAngle.getValue())),

        Commands.run(() -> subIntake.setIntakeMotorSpeed(intakeSpeed)).until(() -> subIntake.isGamePieceCollected()),

        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subIntake.setCurrentGamePiece(gamepiece)));
  }
}
