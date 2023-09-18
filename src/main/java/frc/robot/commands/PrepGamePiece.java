// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DesiredHeight;
import frc.robot.Constants.GamePiece;
import frc.robot.RobotPreferences.prefElevator;
import frc.robot.RobotPreferences.prefWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class PrepGamePiece extends SequentialCommandGroup {
  Elevator subElevator;
  Wrist subWrist;
  Intake subIntake;

  DesiredHeight desiredHeight;
  double desiredPosition;

  public PrepGamePiece(Elevator subElevator, Wrist subWrist, Intake subIntake) {
    this.subElevator = subElevator;
    this.subWrist = subWrist;
    this.subIntake = subIntake;

    if (subIntake.getCurrentGamePiece() == GamePiece.CUBE) {
      switch (subElevator.getDesiredHeight()) {
        case HYBRID:
          desiredPosition = prefElevator.hybridScore.getValue();
          break;
        case MID:
          desiredPosition = prefElevator.midCubeScore.getValue();
          break;
        case HIGH:
          desiredPosition = prefElevator.highCubeScore.getValue();
          break;
        // Do we want to default to something when desiredHeight == NONE?
      }
    } else {
      switch (subElevator.getDesiredHeight()) {
        case HYBRID:
          desiredPosition = prefElevator.hybridScore.getValue();
          break;
        case MID:
          desiredPosition = prefElevator.midConeScore.getValue();
          break;
        case HIGH:
          desiredPosition = prefElevator.highConeScore.getValue();
          break;
        // Do we want to default to something when desiredHeight == NONE?
      }
    }

    addCommands(
        Commands.runOnce(() -> subElevator.setElevatorPosition(desiredPosition)),

        Commands.waitUntil(() -> subElevator.isElevatorAtPosition(desiredPosition) == true),

        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristScoringAngle.getValue())));
  }
}
