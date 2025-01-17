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
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stow extends SequentialCommandGroup {
  Wrist subWrist;
  Intake subIntake;
  Elevator subElevator;
  LEDs subLEDs;

  public Stow(Wrist subWrist, Intake subIntake, Elevator subElevator, LEDs subLEDs) {
    this.subWrist = subWrist;
    this.subIntake = subIntake;
    this.subElevator = subElevator;
    this.subLEDs = subLEDs;

    addRequirements(subWrist, subIntake, subElevator, subLEDs);

    addCommands(
        Commands.runOnce(() -> subLEDs.clearAnimation()),
        Commands.runOnce(() -> subWrist.setWristAngle(prefWrist.wristStowAngle.getValue())),

        Commands.runOnce(() -> subIntake.setIntakeMotorSpeed(0)).unless(() -> subIntake.isGamePieceCollected()),

        Commands.waitUntil(() -> subWrist.isWristAtPosition(prefWrist.wristStowAngle.getValue())),
        Commands.runOnce(() -> subElevator.setElevatorPosition(prefElevator.elevatorStow.getValue())),

        Commands.waitUntil((() -> subElevator.isElevatorAtPosition(prefElevator.elevatorStow.getValue(),
            prefElevator.elevatorActualPositionTolerance.getValue()))),

        Commands.run(() -> subElevator.neutralElevatorOutputs()));
  }
}
