// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class Engage extends CommandBase {

  Drivetrain subDrivetrain;

  boolean isDriveOpenLoop;
  double desiredSpeedFeet;
  double timeElapsed = 0;

  public Engage(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;

    isDriveOpenLoop = false;

    addRequirements(subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (subDrivetrain.isTiltedForward() && timeElapsed == 0) {
      timeElapsed = 1;
      desiredSpeedFeet = prefDrivetrain.forwardTiltDockSpeed.getValue();

    } else if (subDrivetrain.isTiltedBackwards() && timeElapsed == 0) {
      timeElapsed = 1;
      desiredSpeedFeet = prefDrivetrain.backwardTitDockSpeed.getValue();

    } else if (subDrivetrain.isTiltedForward() && timeElapsed != 0) {
      timeElapsed += 0.05;
      desiredSpeedFeet = prefDrivetrain.forwardTiltDockSpeed.getValue() / timeElapsed;

    } else if (subDrivetrain.isTiltedBackwards() && timeElapsed != 0) {
      timeElapsed += 0.05;
      desiredSpeedFeet = prefDrivetrain.backwardTitDockSpeed.getValue() / timeElapsed;

    } else {
      timeElapsed = 1.5;
      desiredSpeedFeet = 0;

    }
    subDrivetrain.drive(
        new Pose2d(Units.metersToFeet(desiredSpeedFeet), 0, new Rotation2d()),
        isDriveOpenLoop);
  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
