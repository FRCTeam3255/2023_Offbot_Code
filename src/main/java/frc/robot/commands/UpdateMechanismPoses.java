// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constWrist;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class UpdateMechanismPoses extends CommandBase {
  Drivetrain subDrivetrain;
  Elevator subElevator;
  Wrist subWrist;
  Vision subVision;

  Pose3d elevatorStagePose;
  Pose3d elevatorCarriagePose;
  Pose3d wristPose;
  Pose3d drivetrainPose;

  Transform3d carriagePosition;

  public UpdateMechanismPoses(Drivetrain subDrivetrain, Elevator subElevator, Wrist subWrist, Vision subVision) {
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subWrist = subWrist;
    this.subVision = subVision;

    // This doesnt actually need subVision but i wasnt sure how else to run this
    // periodically
    addRequirements(subVision);
  }

  // Copied directly from 4738 🦒 who copied from 6328 🤭
  public static synchronized double[] composePose3ds(Pose3d... value) {
    double[] data = new double[value.length * 7];
    for (int i = 0; i < value.length; i++) {
      data[i * 7] = value[i].getX();
      data[i * 7 + 1] = value[i].getY();
      data[i * 7 + 2] = value[i].getZ();
      data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
      data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
      data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
      data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
    }
    return data;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrainPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    elevatorStagePose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    elevatorCarriagePose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    wristPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    SmartDashboard.putNumberArray("Drivetrain Pose3d", composePose3ds(drivetrainPose));
    SmartDashboard.putNumberArray("Elevator Stage 2 Pose3d", composePose3ds(elevatorStagePose));
    SmartDashboard.putNumberArray("Elevator Carriage Pose3d", composePose3ds(elevatorCarriagePose));
    SmartDashboard.putNumberArray("Wrist Pose3d", composePose3ds(wristPose));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainPose = new Pose3d(
        new Translation3d(
            subDrivetrain.getPose2d().getX(),
            subDrivetrain.getPose2d().getY(),
            0),
        new Rotation3d(subDrivetrain.getRoll(), subDrivetrain.getPitch(), subDrivetrain.getYaw()));

    // TODO: UPDATE POSES ACCORDINGLY

    SmartDashboard.putNumberArray("Drivetrain Pose3d", composePose3ds(drivetrainPose));
    SmartDashboard.putNumberArray("Elevator Stage 2 Pose3d", composePose3ds(elevatorStagePose));
    SmartDashboard.putNumberArray("Elevator Carriage Pose3d", composePose3ds(elevatorCarriagePose));
    SmartDashboard.putNumberArray("Wrist Pose3d", composePose3ds(wristPose));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
