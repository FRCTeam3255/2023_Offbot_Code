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

  double desiredWristCone;
  double desiredElevatorCone;
  double desiredWristCube;
  double desiredElevatorCube;

  double desiredPosition;
  double desiredAngle;

  public PrepGamePiece(Elevator subElevator, Wrist subWrist, Intake subIntake,
      double desiredWristCone, double desiredElevatorCone, double desiredWristCube, double desiredElevatorCube) {
    this.subElevator = subElevator;
    this.subWrist = subWrist;
    this.subIntake = subIntake;

    this.desiredWristCone = desiredWristCone;
    this.desiredElevatorCone = desiredElevatorCone;
    this.desiredWristCube = desiredWristCube;
    this.desiredElevatorCube = desiredElevatorCube;

    addRequirements(subElevator, subWrist, subIntake);

    addCommands(
        // --- Determining the desired height and wrist angle ---
        Commands.runOnce(() -> desiredPosition = desiredElevatorCube)
            .unless(() -> !subIntake.getCurrentGamePiece().equals(GamePiece.CUBE)),
        Commands.runOnce(() -> desiredPosition = desiredElevatorCone)
            .unless(() -> subIntake.getCurrentGamePiece().equals(GamePiece.CUBE)),

        Commands.runOnce(() -> desiredAngle = desiredWristCube)
            .unless(() -> !subIntake.getCurrentGamePiece().equals(GamePiece.CUBE)),
        Commands.runOnce(() -> desiredAngle = desiredWristCone)
            .unless(() -> subIntake.getCurrentGamePiece().equals(GamePiece.CUBE)),
        // --- Determining the desired height and wrist angle ---

        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),
        Commands.waitUntil(() -> subWrist.isWristAtPosition(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subElevator.setElevatorPosition(desiredPosition)),

        Commands.waitUntil(() -> subElevator.isElevatorAtPosition(desiredPosition)),

        Commands.runOnce(() -> subWrist.setWristAngle(desiredAngle)),

        Commands.runOnce(() -> subElevator.setIsPrepped(true)));
  }
}
