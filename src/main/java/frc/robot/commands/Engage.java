// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class Engage extends CommandBase {

  Drivetrain subDrivetrain;

  boolean isDriveOpenLoop;
  double desiredSpeedFeet;
  double degreesFromEngaged = 0;
  double timeWhenEngaged;
  double debounce = 0;

  public Engage(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;

    isDriveOpenLoop = false;

    addRequirements(subDrivetrain);
  }

  @Override
  public void initialize() {
    degreesFromEngaged = Math.abs(0 - subDrivetrain.getNavXRoll());
  }

  @Override
  public void execute() {
    degreesFromEngaged = SN_Math.interpolate(Math.abs(0 - subDrivetrain.getNavXRoll()), 0, 15, 0, 1);

    if ((subDrivetrain.isTiltedForward() || subDrivetrain.isTiltedBackwards()) && debounce < 10) {
      debounce += 1;

    } else if (subDrivetrain.isTiltedForward()) {
      desiredSpeedFeet = prefDrivetrain.maxForwardDockSpeed.getValue();
      timeWhenEngaged = 0;
    } else if (subDrivetrain.isTiltedBackwards()) {
      desiredSpeedFeet = -prefDrivetrain.maxBackDockSpeed.getValue();
      timeWhenEngaged = 0;

    } else {
      desiredSpeedFeet = 0;
      debounce = 0;
      timeWhenEngaged = Timer.getFPGATimestamp();

    }

    subDrivetrain.drive(new Pose2d(Units.metersToFeet(desiredSpeedFeet * (degreesFromEngaged * degreesFromEngaged)), 0,
        new Rotation2d()), isDriveOpenLoop);
  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.setDefenseMode();
  }

  @Override
  public boolean isFinished() {
    return timeWhenEngaged != 0
        && timeWhenEngaged < Timer.getFPGATimestamp() - prefDrivetrain.engagedSeconds.getValue();
  }
}
