// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperShuffle;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.DesiredHeight;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.SetLEDs;
import frc.robot.commands.Auto.OnePiece.CenterCube;
import frc.robot.commands.Auto.OnePiece.CubeThenEngageCenter;
import frc.robot.commands.Auto.OnePiece.CubeThenMobilityCable;
import frc.robot.commands.Auto.OnePiece.CubeThenMobilityOpen;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  ShuffleboardTab autoTab = Shuffleboard.getTab("SuperShuffle");

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();
  private final SuperShuffle subSuperShuffle = new SuperShuffle();
  private final Elevator subElevator = new Elevator();
  private final Intake subIntake = new Intake();
  private final Wrist subWrist = new Wrist();
  private final Vision subVision = new Vision();
  private final LEDs subLEDs = new LEDs();

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  private static DigitalInput pracBotSwitch = new DigitalInput(9);
  private final Trigger teleopTrigger = new Trigger(() -> RobotState.isEnabled() && RobotState.isTeleop());

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_X_DEADBAND);

    subDrivetrain
        .setDefaultCommand(new Drive(
            subDrivetrain,
            conDriver.axis_LeftY,
            conDriver.axis_LeftX,
            conDriver.axis_RightX,
            conDriver.axis_RightTrigger,
            conDriver.btn_Y,
            conDriver.btn_B,
            conDriver.btn_A,
            conDriver.btn_X));
    subVision.setDefaultCommand(new AddVisionMeasurement(subDrivetrain,
        subVision));
    subLEDs.setDefaultCommand(new SetLEDs(subLEDs, subDrivetrain, subIntake));

    configureBindings();
    configureAutoSelector();

    Timer.delay(2.5);
    resetToAbsolutePositions();
  }

  /**
   * Reset all the applicable motor encoders to their corresponding absolute
   * encoder.
   */
  public void resetToAbsolutePositions() {
    subDrivetrain.resetSteerMotorEncodersToAbsolute();
  }

  private void configureBindings() {

    // Driver

    // "reset gyro" for field relative but actually resets the orientation at a
    // higher level
    conDriver.btn_Back
        .onTrue(Commands.runOnce(
            () -> subDrivetrain.resetRotation()));

    // while true do robot oriented, default to field oriented
    conDriver.btn_LeftBumper
        .whileTrue(Commands.runOnce(() -> subDrivetrain.setRobotRelative()))
        .onFalse(Commands.runOnce(() -> subDrivetrain.setFieldRelative()));

    conDriver.btn_RightBumper
        .whileTrue(Commands.run(() -> subDrivetrain.setDefenseMode(), subDrivetrain))
        .whileTrue(Commands.run(() -> subLEDs.setLEDPattern(constLEDs.DEFENSE_MODE_COLOR)));

    // Operator

    // Intake Cone (RB)
    conOperator.btn_RightBumper.onTrue(new IntakeGamePiece(subWrist, subIntake, subElevator, GamePiece.CONE));

    // Intake Cube (LB)
    conOperator.btn_LeftBumper.onTrue(new IntakeGamePiece(subWrist, subIntake, subElevator, GamePiece.CUBE));

    // Set desiredHeight to Hybrid (South)
    conOperator.btn_South.onTrue(Commands.runOnce(() -> subElevator.setDesiredHeight(DesiredHeight.HYBRID)));

    // Set desiredHeight to Mid (East or West)
    conOperator.btn_East.onTrue(Commands.runOnce(() -> subElevator.setDesiredHeight(DesiredHeight.MID)));
    conOperator.btn_West.onTrue(Commands.runOnce(() -> subElevator.setDesiredHeight(DesiredHeight.MID)));

    // Set desiredHeight to High (North)
    conOperator.btn_North.onTrue(Commands.runOnce(() -> subElevator.setDesiredHeight(DesiredHeight.HIGH)));

    // teleopTrigger.onTrue(new SetRumble(conDriver, conOperator, subIntake));
  }

  public static boolean isPracticeBot() {
    return !pracBotSwitch.get();
  }

  private void configureAutoSelector() {
    autoChooser.setDefaultOption("null", null);

    autoChooser.addOption("Score Cube Then Mobility Cable",
        new CubeThenMobilityCable(subDrivetrain));
    autoChooser.addOption("Score Cube Then Engage Center", new CubeThenEngageCenter(subDrivetrain));
    autoChooser.addOption("Score Cube Center (NO DOCK)", new CenterCube(subDrivetrain));
    autoChooser.addOption("Score Cube Then Mobility Open", new CubeThenMobilityOpen(subDrivetrain));

    SmartDashboard.putData(autoChooser);

    autoTab
        .add("Auto Chooser", autoChooser)
        .withSize(2, 1)
        .withPosition(7, 2);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
