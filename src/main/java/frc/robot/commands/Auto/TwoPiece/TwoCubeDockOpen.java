// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.TwoPiece;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamePiece;
import frc.robot.RobotPreferences.prefElevator;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefWrist;
import frc.robot.commands.Engage;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.PrepGamePiece;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoCubeDockOpen extends SequentialCommandGroup {

  Drivetrain subDrivetrain;
  Intake subIntake;
  Wrist subWrist;
  Elevator subElevator;
  LEDs subLEDs;

  public TwoCubeDockOpen(Drivetrain subDrivetrain, Intake subIntake, Wrist subWrist, Elevator subElevator,
      LEDs subLEDs) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subWrist = subWrist;
    this.subElevator = subElevator;
    this.subLEDs = subLEDs;

    addCommands(
        Commands.runOnce(() -> subDrivetrain.resetRotation()),
        Commands.runOnce(() -> subDrivetrain.setNavXAngleAdjustment(
            subDrivetrain.scoreToCubeOpen.getInitialHolonomicPose().getRotation().getDegrees())),

        // Intake cube
        Commands.runOnce(() -> subIntake.setDesiredGamePiece(GamePiece.CUBE)),

        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(prefIntake.intakeCubeSpeed.getValue()))
            .until(() -> subIntake.isGamePieceCollected()).withTimeout(5),

        // Place cube and stow
        new PrepGamePiece(subElevator, subWrist, subIntake,
            prefWrist.wristScoreHighConeAngle.getValue(), prefElevator.elevatorHighConeScore.getValue(),
            prefWrist.wristScoreHighCubeAngle.getValue(), prefElevator.elevatorHighCubeScore.getValue()),

        Commands.waitUntil(() -> subElevator.isPrepped()),

        Commands.waitSeconds(0.2),

        new PlaceGamePiece(subIntake, subWrist, subElevator, false),

        Commands.waitUntil(() -> !subElevator.isPrepped()),

        // Drive to collect a cube
        RobotContainer.swerveAutoBuilder.fullAuto(subDrivetrain.scoreToCubeOpen)
            .withTimeout(subDrivetrain.scoreToCubeOpen.getTotalTimeSeconds()),

        new PlaceGamePiece(subIntake, subWrist, subElevator, true),

        Commands.waitUntil(() -> !subElevator.isPrepped()),

        new Engage(subDrivetrain));

    // Drive to score
    // Commands.race(
    // RobotContainer.swerveAutoBuilder.fullAuto(subDrivetrain.cubeToScoreOpen)
    // .withTimeout(subDrivetrain.cubeToScoreOpen.getTotalTimeSeconds())));

    // // Place cube
    // new PrepGamePiece(subElevator, subWrist, subIntake,
    // prefWrist.wristScoreHighConeAngle.getValue(),
    // prefElevator.elevatorHighConeScore.getValue(),
    // prefWrist.wristScoreHighCubeAngle.getValue(),
    // prefElevator.elevatorHighCubeScore.getValue()),

    // Commands.waitUntil(() -> subElevator.isPrepped()),

    // new PlaceGamePiece(subIntake, subWrist, subElevator),

    // Commands.waitUntil(() -> !subElevator.isPrepped()),

    // // Engage
    // new Engage(subDrivetrain));
  }
}
