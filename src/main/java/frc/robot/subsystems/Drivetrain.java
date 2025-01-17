// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AdvantageScopeUtil;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.SN_SwerveModule;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefVision;

public class Drivetrain extends SubsystemBase {

  ShuffleboardTab tab = Shuffleboard.getTab("SuperShuffle");

  private SN_SwerveModule[] modules;

  private AHRS navX;

  public SwerveDriveKinematics swerveKinematics;

  private SwerveDrivePoseEstimator poseEstimator;

  private boolean isFieldRelative;

  private Field2d field;

  private ProfiledPIDController xPID;
  private ProfiledPIDController yPID;
  private PIDController thetaPID;

  double[] swerveRealStates = new double[8];
  double[] swerveDesiredStates = new double[8];
  double simAngle = 0;
  SwerveModuleState[] lastDesiredStates;
  private double timeFromLastUpdate;
  private double lastSimTime;
  private Timer simTimer;

  public Double[] columnYCoordinatesBlue = { 0.5, 1.05, 1.63, 2.19, 2.75, 3.31, 3.86, 4.43, 4.98 };
  public Double[] columnYCoordinatesRed = { 4.98, 4.43, 3.86, 3.31, 2.75, 2.19, 1.63, 1.05, 0.5 };

  // Paths
  public PathPlannerTrajectory testLinePath;
  public PathPlannerTrajectory centerCoDock;
  public PathPlannerTrajectory cableCoCu;
  public PathPlannerTrajectory cableCoCuDock;
  public PathPlannerTrajectory cableCoCoYeetDock;
  public PathPlannerTrajectory cableCoCoCoYeetDock;
  public PathPlannerTrajectory openCoCu;
  public PathPlannerTrajectory openCoCuDock;
  public PathPlannerTrajectory openCoCoCoYeetDock;
  public PathPlannerTrajectory just_taxi;

  public Drivetrain() {
    if (Robot.isSimulation()) {
      simTimer = new Timer();
      simTimer.start();
      lastSimTime = simTimer.get();
      timeFromLastUpdate = 0;
    }

    if (RobotContainer.isPracticeBot()) {
      modules = new SN_SwerveModule[] {
          new SN_SwerveModule(Constants.PRAC_MODULE_0),
          new SN_SwerveModule(Constants.PRAC_MODULE_1),
          new SN_SwerveModule(Constants.PRAC_MODULE_2),
          new SN_SwerveModule(Constants.PRAC_MODULE_3)
      };

      swerveKinematics = Constants.PRAC_SWERVE_KINEMATICS;
    } else {
      modules = new SN_SwerveModule[] {
          new SN_SwerveModule(Constants.MODULE_0),
          new SN_SwerveModule(Constants.MODULE_1),
          new SN_SwerveModule(Constants.MODULE_2),
          new SN_SwerveModule(Constants.MODULE_3)
      };

      swerveKinematics = Constants.SWERVE_KINEMATICS;
    }

    navX = new AHRS(SPI.Port.kMXP);
    Timer.delay(1);
    navX.reset();

    poseEstimator = new SwerveDrivePoseEstimator(
        swerveKinematics,
        getRotation2dYaw(),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(
            Units.feetToMeters(prefDrivetrain.measurementStdDevsFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.measurementStdDevsFeet.getValue()),
            Units.degreesToRadians(prefDrivetrain.measurementStdDevsDegrees.getValue())),
        VecBuilder.fill(
            Units.feetToMeters(prefVision.measurementStdDevsFeet.getValue()),
            Units.feetToMeters(prefVision.measurementStdDevsFeet.getValue()),
            Units.degreesToRadians(prefVision.measurementStdDevsDegrees.getValue())));

    isFieldRelative = true;

    field = new Field2d();

    xPID = new ProfiledPIDController(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.feetToMeters(prefDrivetrain.teleTransMaxSpeed.getValue()),
            Units.feetToMeters(prefDrivetrain.teleTransMaxAccel.getValue())));

    yPID = new ProfiledPIDController(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.feetToMeters(prefDrivetrain.teleTransMaxSpeed.getValue()),
            Units.feetToMeters(prefDrivetrain.teleTransMaxAccel.getValue())));

    thetaPID = new PIDController(
        prefDrivetrain.teleThetaP.getValue(),
        prefDrivetrain.teleThetaI.getValue(),
        prefDrivetrain.teleThetaD.getValue());

    testLinePath = PathPlanner.loadPath("testLinePath",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));

    centerCoDock = PathPlanner.loadPath("centerCoDock",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));

    // Cable
    cableCoCuDock = PathPlanner.loadPath("cableCoCuDock",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue() * 1.3),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue() * 1.3)));

    cableCoCu = PathPlanner.loadPath("cableCoCu",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));

    cableCoCoYeetDock = PathPlanner.loadPath("cableCoCoYeetDock",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));

    cableCoCoCoYeetDock = PathPlanner.loadPath("cableCoCoCoYeetDock",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue() * 1.3),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue() * 1.3)));

    // Open
    openCoCuDock = PathPlanner.loadPath("openCoCuDock",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue() * 1.3),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue() * 1.3)));

    openCoCu = PathPlanner.loadPath("openCoCu",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));

    openCoCoCoYeetDock = PathPlanner.loadPath("openCoCoCoYeetDock",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue() * 1.3),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue() * 1.3)));

    just_taxi = PathPlanner.loadPath("just_taxi",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));

    configure();
  }

  public void configure() {
    for (SN_SwerveModule mod : modules) {
      mod.configure();
    }

    xPID.setPID(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue());
    xPID.setConstraints(new TrapezoidProfile.Constraints(
        Units.feetToMeters(prefDrivetrain.teleTransMaxSpeed.getValue()),
        Units.feetToMeters(prefDrivetrain.teleTransMaxAccel.getValue())));
    xPID.setTolerance(Units.inchesToMeters(prefDrivetrain.teleTransTolerance.getValue()));
    xPID.reset(getPose2d().getX());

    yPID.setPID(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue());
    yPID.setConstraints(new TrapezoidProfile.Constraints(
        Units.feetToMeters(prefDrivetrain.teleTransMaxSpeed.getValue()),
        Units.feetToMeters(prefDrivetrain.teleTransMaxAccel.getValue())));
    yPID.setTolerance(Units.inchesToMeters(prefDrivetrain.teleThetaTolerance.getValue()));
    yPID.reset(getPose2d().getY());

    thetaPID.setPID(
        prefDrivetrain.teleThetaP.getValue(),
        prefDrivetrain.teleThetaI.getValue(),
        prefDrivetrain.teleThetaD.getValue());
    thetaPID.setTolerance(Units.inchesToMeters(prefDrivetrain.teleThetaTolerance.getValue()));
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    thetaPID.reset();

  }

  /**
   * Drive the drivetrain to a specified position in meters.
   * 
   * @param position Desired position in meters
   */
  public void driveToPosition(Pose2d position) {

    // tell the x and y PID controllers the goal position.
    xPID.setGoal(position.getX());
    yPID.setGoal(position.getY());

    // create a velocity Pose2d with the calculated x and y positions, and the
    // positional rotation.
    Pose2d velocity = new Pose2d(
        xPID.calculate(getPose2d().getX()),
        yPID.calculate(getPose2d().getY()),
        position.getRotation());

    // pass the velocity Pose2d to driveAlignAngle(), which will close the loop for
    // rotation and pass the translational values to drive().
    driveAlignAngle(velocity, false);
  }

  /**
   * Drive the drivetrain with positional absolute heading control.
   * 
   * @param velocity Desired translational velocity in meters per second, and
   *                 desired absolute rotational position
   */
  public void driveAlignAngle(Pose2d velocity, boolean isDriveOpenLoop) {

    // tell the theta PID controller the goal rotation.
    thetaPID.setSetpoint(velocity.getRotation().getRadians());

    // calculate the angle setpoint based off where we are now.
    double angleSetpoint = thetaPID.calculate(getRotation().getRadians());

    // limit the PID output to a maximum rotational speed
    angleSetpoint = MathUtil.clamp(angleSetpoint, -Units.degreesToRadians(prefDrivetrain.teleThetaMaxSpeed.getValue()),
        Units.degreesToRadians(prefDrivetrain.teleThetaMaxSpeed.getValue()));

    // create a new velocity Pose2d with the same translation as the on that was
    // passed in, but with the output of the theta PID controller for rotation.
    Pose2d newVelocity = new Pose2d(velocity.getTranslation(), Rotation2d.fromRadians(angleSetpoint));

    // pass the new velocity to the normal drive command
    drive(newVelocity, isDriveOpenLoop);
  }

  /**
   * Drive the drivetrain
   * 
   * @param velocity Desired translational and rotational velocity in meters per
   *                 second and radians per second
   */
  public void drive(Pose2d velocity, boolean isDriveOpenLoop) {

    ChassisSpeeds chassisSpeeds;

    if (isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          velocity.getX(),
          velocity.getY(),
          velocity.getRotation().getRadians(),
          getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(
          velocity.getX(),
          velocity.getY(),
          velocity.getRotation().getRadians());
    }

    SwerveModuleState[] desiredStates;

    desiredStates = swerveKinematics.toSwerveModuleStates(discretize(chassisSpeeds));

    setModuleStates(desiredStates, isDriveOpenLoop);
  }

  /**
   * Set each module state
   * 
   * @param desiredStates Array of desired states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isDriveOpenLoop) {
    lastDesiredStates = desiredStates;

    // desaturateWheelSpeeds() mutates the given array
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_MODULE_SPEED);

    for (int i = 0; i < 8; i += 2) {
      swerveDesiredStates[i] = desiredStates[i / 2].angle.getRadians();
      swerveDesiredStates[i + 1] = desiredStates[i / 2].speedMetersPerSecond;
    }

    for (SN_SwerveModule mod : modules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], isDriveOpenLoop, false);
    }
  }

  public void setModuleStatesAuto(SwerveModuleState[] desiredStates) {
    setModuleStates(desiredStates, false);
  }

  /**
   * Turn all the wheels inward to be better against defense.
   */
  public void setDefenseMode() {
    SwerveModuleState[] desiredStates = {
        new SwerveModuleState(0, Constants.MODULE_0_DEFENSE_ANGLE),
        new SwerveModuleState(0, Constants.MODULE_1_DEFENSE_ANGLE),
        new SwerveModuleState(0, Constants.MODULE_2_DEFENSE_ANGLE),
        new SwerveModuleState(0, Constants.MODULE_3_DEFENSE_ANGLE) };

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_MODULE_SPEED);

    for (SN_SwerveModule mod : modules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], true, true);
    }
  }

  public void neutralDriveOutputs() {
    for (SN_SwerveModule mod : modules) {
      mod.neutralDriveOutput();
    }
  }

  /**
   * Reset the steer motor encoders to the absolute encoders.
   */
  public void resetSteerMotorEncodersToAbsolute() {
    for (SN_SwerveModule mod : modules) {
      mod.resetSteerMotorEncodersToAbsolute();
    }
  }

  /**
   * Set the drive method to use field relative drive controls
   */
  public void setFieldRelative() {
    isFieldRelative = true;
  }

  /**
   * Set the drive method to use robot relative drive controls
   */
  public void setRobotRelative() {
    isFieldRelative = false;
  }

  public void resetPID() {
    xPID.reset(getPose2d().getX());
    yPID.reset(getPose2d().getY());
    thetaPID.reset();
  }

  /**
   * Get the current estimated 2d position of the drivetrain.
   * 
   * @return Position of drivetrain
   */
  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Get the rotation of the drivetrain. This method currently just uses the navX
   * yaw, but this is subject to change.
   * 
   * @return Rotation of drivetrain
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromRadians(MathUtil.angleModulus(getRotation2dYaw().getRadians()));
  }

  /**
   * Reset the rotation of the drivetrain to zero. This method currently just
   * resets the navX yaw, but this is subject to change.
   */
  public void resetRotation() {
    navX.reset();
  }

  public void setNavXAngleAdjustment(double adjustment) {
    navX.setAngleAdjustment(adjustment);
  }

  /**
   * Get an array of each module position. A position consists of a distance and
   * angle.
   * 
   * @return Array of swerve module positions
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SN_SwerveModule mod : modules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

  /**
   * Updates the pose estimator with the current robot uptime, the gyro yaw, and
   * each swerve module position.
   * <p>
   * This method MUST be called every loop (or else pose estimator breaks)
   */
  public void updatePoseEstimator() {
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        getRotation2dYaw(),
        getModulePositions());

  }

  /**
   * Adds a new vision measurement to the pose estimator.
   *
   * @param visionMeasurement The pose measurement from the vision system
   * @param timestampSeconds  The timestamp of the measurement in seconds
   */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(
        visionMeasurement,
        timestampSeconds,
        VecBuilder.fill(
            Units.feetToMeters(prefVision.measurementStdDevsFeet.getValue()),
            Units.feetToMeters(prefVision.measurementStdDevsFeet.getValue()),
            Units.degreesToRadians(prefVision.measurementStdDevsDegrees.getValue())));
  }

  /**
   * Reset pose estimator to given position
   * 
   * @param pose Position to reset to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
        getRotation2dYaw(),
        getModulePositions(),
        pose);
  }

  public double getNavXRoll() {
    return navX.getRoll();
  }

  public double getRoll() {
    return navX.getRoll();
  }

  public double getPitch() {
    return navX.getPitch();
  }

  public double getYaw() {
    return navX.getYaw();
  }

  public Rotation2d getRotation2dYaw() {
    if (Robot.isSimulation() && lastDesiredStates != null) {
      timeFromLastUpdate = simTimer.get() - lastSimTime;
      lastSimTime = simTimer.get();
      simAngle += swerveKinematics.toChassisSpeeds(lastDesiredStates).omegaRadiansPerSecond * timeFromLastUpdate;
      return new Rotation2d(simAngle);
    }
    return navX.getRotation2d();
  }

  public boolean isNavXConnected() {
    return navX.isConnected();
  }

  /**
   * This is the most theoretical thing that is in the code.
   * It takes our current position and then adds an offset to it, knowing that the
   * robot's estimated position
   * is not following the exact position of the robot.
   * 
   * @param speeds the speeds about to be inputted into the robot.
   * @return the same thing as we input.
   *         Think of this method as an interceptor,
   *         not changing the parameter but using it for calculations.
   */
  /**
   * Credit: WPIlib 2024 and 4738
   * Discretizes a continuous-time chassis speed.
   *
   * @param vx    Forward velocity.
   * @param vy    Sideways velocity.
   * @param omega Angular velocity.
   */
  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;

    var desiredDeltaPose = new Pose2d(
        speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt,
        new Rotation2d(speeds.omegaRadiansPerSecond * dt * 4));

    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  @Override
  public void periodic() {

    updatePoseEstimator();

    SmartDashboard.putBoolean("Drivetrain/Field Relative", isFieldRelative);

    if (Constants.OUTPUT_DEBUG_VALUES) {

      SmartDashboard.putNumber("Drivetrain/Pose X", Units.metersToInches(getPose2d().getX()));
      SmartDashboard.putNumber("Drivetrain/Pose Y", Units.metersToInches(getPose2d().getY()));
      SmartDashboard.putNumber("Drivetrain/Pose Rotation", getPose2d().getRotation().getDegrees());

      SmartDashboard.putNumber("Drivetrain/Yaw", getRotation2dYaw().getDegrees());
      SmartDashboard.putNumber("Drivetrain/Roll", getRoll());
      SmartDashboard.putBoolean("Drivetrain/NavX Connected", isNavXConnected());

      SmartDashboard.putNumber("Drivetrain/Theta Goal", Units.radiansToDegrees(thetaPID.getSetpoint()));
      SmartDashboard.putNumber("Drivetrain/Theta Error",
          Units.radiansToDegrees(thetaPID.getPositionError()));

      field.setRobotPose(getPose2d());
      SmartDashboard.putData(field);

      for (SN_SwerveModule mod : modules) {
        SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Speed",
            Units.metersToFeet(mod.getState().speedMetersPerSecond));
        SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Distance",
            Units.metersToFeet(mod.getPosition().distanceMeters));
        SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Angle",
            mod.getState().angle.getDegrees());
        SmartDashboard.putBoolean("Drivetrain/Module " + mod.moduleNumber + "/Absolute Encoder Unplugged",
            mod.getAbsoluteEncoderUnplugged());
        SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Absolute Encoder Angle",
            mod.getAbsoluteEncoder().getDegrees());
        SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Raw Absolute Encoder Angle",
            mod.getRawAbsoluteEncoder());
        SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Drive Output Percent",
            mod.getDriveMotorOutputPercent());
      }

    }

    for (int i = 0; i < 8; i += 2) {
      swerveRealStates[i] = modules[i / 2].getState().angle.getRadians();
      swerveRealStates[i + 1] = modules[i / 2].getState().speedMetersPerSecond;
    }

    SmartDashboard.putNumberArray("Drivetrain/DesiredStates", swerveDesiredStates);
    SmartDashboard.putNumberArray("Drivetrain/RealStates", swerveRealStates);
    SmartDashboard.putNumberArray("Drivetrain/Pose3d",
        AdvantageScopeUtil.composePose3ds(new Pose3d(getPose2d())));
  }
}
