// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.constLEDs;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class SetLEDs extends CommandBase {
  LEDs subLEDs;
  Drivetrain subDrivetrain;
  Intake subIntake;

  int[] desiredPattern = { 0, 0, 0 };

  int desiredColumn;

  public SetLEDs(LEDs subLEDs, Drivetrain subDrivetrain, Intake subIntake) {
    this.subLEDs = subLEDs;
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;

    addRequirements(subLEDs);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    desiredPattern = constLEDs.DEFAULT_COLOR;
    if (subIntake.isGamePieceCollected()) {
      subLEDs.clearAnimation();
      switch (subIntake.getDesiredGamePiece()) {
        case CUBE:
          desiredPattern = constLEDs.HAS_CUBE_COLOR;
          break;
        default:
          desiredPattern = constLEDs.HAS_CONE_COLOR;
          break;
      }
      subLEDs.setLEDs(desiredPattern);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
