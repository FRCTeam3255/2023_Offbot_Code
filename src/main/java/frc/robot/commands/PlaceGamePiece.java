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

    if (subIntake.getCurrentGamePiece() == GamePiece.CUBE) {
      speed = prefIntake.intakePlaceCubeSpeed.getValue();
    } else if (subIntake.getCurrentGamePiece() == GamePiece.CONE) {
      speed = prefIntake.intakePlaceConeSpeed.getValue();
    }

    addCommands(
        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(speed)),

        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subElevator.setElevatorPosition(prefElevator.elevatorStow.getValue())));
  }
}
