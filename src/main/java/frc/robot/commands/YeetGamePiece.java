// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefElevator;
import frc.robot.RobotPreferences.prefWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class YeetGamePiece extends SequentialCommandGroup {
  Elevator subElevator;
  Intake subIntake;
  Wrist subWrist;

  public YeetGamePiece(Intake subIntake, Elevator subElevator, Wrist subWrist) {
    this.subElevator = subElevator;
    this.subIntake = subIntake;
    this.subWrist = subWrist;

    addRequirements(subElevator, subWrist, subIntake);

    addCommands(
        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),
        Commands.waitUntil(() -> subWrist.isWristAtPosition(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subElevator.setElevatorPosition(prefElevator.elevatorHighConeScore.getValue())),

        Commands
            .waitUntil(
                () -> subElevator.isElevatorAtPosition(prefElevator.elevatorYeetGamePiecePosition.getValue(),
                    prefElevator.elevatorActualPositionTolerance.getValue())),

        new PlaceGamePiece(subIntake, subWrist, subElevator, true));
  }
}
