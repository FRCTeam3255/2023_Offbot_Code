// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.preferences.SN_Preferences;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // Run when the robot is first started up. Should be used for any
  // initialization code.
  @Override
  public void robotInit() {

    if (RobotPreferences.useNetworkTables) {
      SN_Preferences.usePreferences();
    } else {
      SN_Preferences.useDefaults();
    }

    CameraServer.startAutomaticCapture();

    m_robotContainer = new RobotContainer();

    Shuffleboard.selectTab("SuperShuffle");
    RobotContainer.subElevator.resetElevatorEncoderToAbsolute();
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling
    CommandScheduler.getInstance().run();

    SmartDashboard.putBoolean("*NT Prefs*", RobotPreferences.useNetworkTables);
    SmartDashboard.putBoolean("*Prac Bot*", RobotContainer.isPracticeBot());
    RobotContainer.logPDHValues();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.subDrivetrain.setDefenseMode();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.resetToAbsolutePositions();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.resetToAbsolutePositions();

    // Comment this line out if you would like autonomous to continue until being
    // interrupted by another command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.subElevator.neutralElevatorOutputs();
    RobotContainer.subWrist.neutralElevatorOutputs();
    RobotContainer.subIntake.setCurrentLimiting(true);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
