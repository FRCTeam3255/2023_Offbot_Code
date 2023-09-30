// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.OnePiece;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DesiredHeight;
import frc.robot.Constants.GamePiece;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefWrist;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.PrepGamePiece;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterCube extends SequentialCommandGroup {

  Drivetrain subDrivetrain;
  Intake subIntake;
  Elevator subElevator;
  Wrist subWrist;

  public CenterCube(Drivetrain subDrivetrain, Intake subIntake, Elevator subElevator, Wrist subWrist) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subElevator = subElevator;
    this.subWrist = subWrist;

    addCommands(
        Commands.runOnce(() -> subDrivetrain.resetRotation()),
        Commands.runOnce(() -> subDrivetrain.setNavXAngleAdjustment(
            subDrivetrain.scoreThenDock.getInitialHolonomicPose().getRotation().getDegrees())),

        Commands.runOnce(() -> subIntake.setCurrentGamePiece(GamePiece.CUBE)),

        Commands.runOnce(() -> subElevator.setDesiredHeight(DesiredHeight.MID)),

        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(prefIntake.intakeCubeSpeed.getValue()))
            .until(() -> subIntake.isGamePieceCollected()).withTimeout(5),

        new PrepGamePiece(subElevator, subWrist, subIntake),

        Commands.waitUntil(() -> subWrist.isWristAtAngle(prefWrist.wristScoringAngle.getValue())),

        new PlaceGamePiece(subIntake, subWrist, subElevator));
  }
}
