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
  double desiredWristAngle = prefWrist.wristScoreAngle.getValue();
  double desiredPosition;

  public PrepGamePiece(Elevator subElevator, Wrist subWrist, Intake subIntake, DesiredHeight desiredHeight) {
    this.subElevator = subElevator;
    this.subWrist = subWrist;
    this.subIntake = subIntake;
    this.desiredHeight = desiredHeight;

    addRequirements(subElevator, subWrist, subIntake);

    addCommands(
        // --- Determining the desired height and wrist angle ---
        // CONE
        Commands.runOnce(() -> desiredPosition = prefElevator.hybridConeScore.getValue())
            .unless(() -> !desiredHeight.equals(DesiredHeight.HYBRID)
                || !subIntake.getCurrentGamePiece().equals(GamePiece.CONE)),

        Commands.runOnce(() -> desiredPosition = prefElevator.midConeScore.getValue())
            .unless(() -> !desiredHeight.equals(DesiredHeight.MID)
                || !subIntake.getCurrentGamePiece().equals(GamePiece.CONE)),
        Commands.runOnce(() -> desiredWristAngle = prefWrist.wristScoreMidConeAngle.getValue())
            .unless(() -> !desiredHeight.equals(DesiredHeight.MID)
                || !subIntake.getCurrentGamePiece().equals(GamePiece.CONE)),

        Commands.runOnce(() -> desiredPosition = prefElevator.highConeScore.getValue())
            .unless(() -> !desiredHeight.equals(DesiredHeight.HIGH)
                || !subIntake.getCurrentGamePiece().equals(GamePiece.CONE)),

        // CUBE
        Commands.runOnce(() -> desiredPosition = prefElevator.hybridCubeScore.getValue())
            .unless(() -> !desiredHeight.equals(DesiredHeight.HYBRID)
                || !subIntake.getCurrentGamePiece().equals(GamePiece.CUBE)),
        Commands.runOnce(() -> desiredWristAngle = prefWrist.wristScoreHybridCubeAngle.getValue())
            .unless(() -> !desiredHeight.equals(DesiredHeight.HYBRID)
                || !subIntake.getCurrentGamePiece().equals(GamePiece.CUBE)),

        Commands.runOnce(() -> desiredPosition = prefElevator.midCubeScore.getValue())
            .unless(() -> !desiredHeight.equals(DesiredHeight.MID)
                || !subIntake.getCurrentGamePiece().equals(GamePiece.CUBE)),
        Commands.runOnce(() -> desiredWristAngle = prefWrist.wristStowAngle.getValue())
            .unless(() -> !desiredHeight.equals(DesiredHeight.MID)
                || !subIntake.getCurrentGamePiece().equals(GamePiece.CUBE)),

        Commands.runOnce(() -> desiredPosition = prefElevator.highCubeScore.getValue())
            .unless(() -> !desiredHeight.equals(DesiredHeight.HIGH)
                || !subIntake.getCurrentGamePiece().equals(GamePiece.CUBE)),
        Commands.runOnce(() -> desiredWristAngle = prefWrist.wristScoreHighCubeAngle.getValue())
            .unless(() -> !desiredHeight.equals(DesiredHeight.HIGH)
                || !subIntake.getCurrentGamePiece().equals(GamePiece.CUBE)),

        // --- Determining the desired height and wrist angle ---

        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),
        Commands.waitUntil(() -> subWrist.isWristAtPosition(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subElevator.setElevatorPosition(desiredPosition)),

        Commands.waitUntil(() -> subElevator.isElevatorAtPosition(desiredPosition)),

        Commands.runOnce(() -> subWrist.setWristAngle(desiredWristAngle)),

        Commands.runOnce(() -> subElevator.setIsPrepped(true)));
  }
}
