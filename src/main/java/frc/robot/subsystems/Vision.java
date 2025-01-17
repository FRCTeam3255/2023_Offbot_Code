// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;

public class Vision extends SubsystemBase {
  PhotonPoseEstimator ARCameraPoseEstimator;
  PhotonPoseEstimator OVCameraPoseEstimator;
  AprilTagFieldLayout aprilTagFieldLayout;

  public Vision() {
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(
          Filesystem.getDeployDirectory().toPath().resolve("ApriltagLocations.json"));
    } catch (Exception e) {
      System.out.println("Could not load AprilTagFieldLayout! Must be on WPILIB 2023.2.1+" + e);
    }

    // PhotonCamera ARCamera = new PhotonCamera(constVision.AR_PHOTON_NAME);
    // Transform3d robotToAR = constVision.ROBOT_TO_AR;

    // PhotonCamera OVCamera = new PhotonCamera(constVision.OV_PHOTON_NAME);
    Transform3d robotToOV = constVision.ROBOT_TO_OV;

    // PhotonCamera lifecam = new PhotonCamera(constVision.LIFECAM_PHOTON_NAME);
    // Transform3d robotToLifecam = constVision.ROBOT_TO_LIFECAM;

    // Create an instance of this for every camera you want to do pose estimation
    // with, as well as a getPoseFrom__ method to reference in the
    // AddVisionMeasurement command
    // ARCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.LOWEST_AMBIGUITY,
    // ARCamera,
    // robotToAR);

    // OVCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.LOWEST_AMBIGUITY,
    // OVCamera,
    // robotToOV);
  }

  public Optional<EstimatedRobotPose> getPoseFromARCamera() {
    try {
      return ARCameraPoseEstimator.update();
    } catch (Exception e) {
      return null;
    }
  }

  public Optional<EstimatedRobotPose> getPoseFromOVCamera() {
    try {
      return OVCameraPoseEstimator.update();
    } catch (Exception e) {
      return null;
    }
  }

  @Override
  public void periodic() {
  }
}
