// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotPreferences.prefElevator;
import frc.robot.RobotPreferences.prefWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stow extends ParallelCommandGroup {
  Wrist subWrist;
  Intake subIntake;
  Elevator subElevator;

  public Stow(Wrist subWrist, Intake subIntake, Elevator subElevator) {
    this.subWrist = subWrist;
    this.subIntake = subIntake;
    this.subElevator = subElevator;

    addRequirements(subWrist, subIntake, subElevator);

    addCommands(
        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(0)).unless(() -> subIntake.isGamePieceCollected()),

        Commands.runOnce(() -> subElevator.setElevatorPosition(prefElevator.elevatorStow.getValue())));
  }
}
