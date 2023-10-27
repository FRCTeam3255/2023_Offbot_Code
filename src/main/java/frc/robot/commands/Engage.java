// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.Constants.constLEDs;

public class Engage extends CommandBase {

  Drivetrain subDrivetrain;
  LEDs subLEDs;

  boolean isDriveOpenLoop;
  double desiredSpeedFeet;
  PIDController rollPID;

  public Engage(Drivetrain subDrivetrain, LEDs subLEDs) {
    this.subDrivetrain = subDrivetrain;
    this.subLEDs = subLEDs;

    isDriveOpenLoop = false;

    rollPID = new PIDController(
        prefDrivetrain.autoEngageP.getValue(),
        prefDrivetrain.autoEngageI.getValue(),
        prefDrivetrain.autoEngageD.getValue());

    rollPID.setTolerance(prefDrivetrain.autoEngageTolerance.getValue());

    addRequirements(subDrivetrain, subLEDs);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Value is negative because we need to go in the opposite direction
    desiredSpeedFeet = -((rollPID.calculate(subDrivetrain.getNavXRoll(), 0))
        * prefDrivetrain.autoEngageSpeedMultiplier.getValue()) * prefDrivetrain.autoMaxSpeedFeet.getValue();

    if (rollPID.atSetpoint()) {
      subDrivetrain.setDefenseMode();
      subLEDs.setLEDPattern(constLEDs.DEFENSE_MODE_COLOR);
    } else {
      subDrivetrain.drive(new Pose2d(Units.metersToFeet(desiredSpeedFeet), 0,
          new Rotation2d()), isDriveOpenLoop);
      subLEDs.setLEDPattern(constLEDs.DEFAULT_COLOR);

    }
  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.setDefenseMode();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
