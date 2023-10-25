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

public class Engage extends CommandBase {

  Drivetrain subDrivetrain;

  boolean isDriveOpenLoop;
  double desiredSpeedFeet;
  PIDController rollPID;

  public Engage(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;

    isDriveOpenLoop = false;

    rollPID = new PIDController(
        prefDrivetrain.autoEngageP.getValue(),
        prefDrivetrain.autoEngageI.getValue(),
        prefDrivetrain.autoEngageD.getValue());

    rollPID.setTolerance(prefDrivetrain.autoEngageTolerance.getValue());

    addRequirements(subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Value is negative because we need to go in the opposite direction
    desiredSpeedFeet = -((rollPID.calculate(subDrivetrain.getNavXRoll(), 0))
        * prefDrivetrain.autoEngageSpeedMultiplier.getValue()) * prefDrivetrain.autoMaxSpeedFeet.getValue();

    subDrivetrain.drive(new Pose2d(Units.metersToFeet(desiredSpeedFeet), 0,
        new Rotation2d()), isDriveOpenLoop);
  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.setDefenseMode();
  }

  @Override
  public boolean isFinished() {
    return rollPID.atSetpoint();
  }
}
