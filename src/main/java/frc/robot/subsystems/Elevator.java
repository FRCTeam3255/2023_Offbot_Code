// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;
import frc.robot.AdvantageScopeUtil;
import frc.robot.Robot;
import frc.robot.Constants.DesiredHeight;
import frc.robot.RobotMap.mapElevator;
import frc.robot.RobotPreferences.prefElevator;

public class Elevator extends SubsystemBase {

  TalonFX leftMotor;
  TalonFX rightMotor;

  DutyCycleEncoder absoluteEncoder;
  double absoluteEncoderOffset;

  TalonFXConfiguration config;
  SupplyCurrentLimitConfiguration supplyLimit;

  DesiredHeight desiredHeight;
  double desiredPosition;
  boolean isPrepped;

  Pose3d elevatorStagePose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  Pose3d elevatorCarriagePose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

  public Elevator() {
    leftMotor = new TalonFX(mapElevator.LEFT_MOTOR_CAN);
    rightMotor = new TalonFX(mapElevator.RIGHT_MOTOR_CAN);
    config = new TalonFXConfiguration();

    absoluteEncoder = new DutyCycleEncoder(mapElevator.ELEVATOR_ABSOLUTE_ENCODER_DIO);
    absoluteEncoderOffset = constElevator.ABSOLUTE_ENCODER_OFFSET;

    desiredHeight = DesiredHeight.NONE;

    configure();
  }

  public void configure() {
    if (absoluteEncoder.getAbsolutePosition() < constElevator.ABSOLUTE_ENCODER_ROLLOVER_OFFSET) {
      absoluteEncoder.setPositionOffset(1);
    }

    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    config.slot0.kP = prefElevator.elevatorP.getValue();
    config.slot0.kI = prefElevator.elevatorI.getValue();
    config.slot0.kD = prefElevator.elevatorD.getValue();

    config.slot0.allowableClosedloopError = SN_Math.metersToFalcon(prefElevator.elevatorPIDTolerance.getValue(),
        constElevator.CIRCUMFERENCE, constElevator.GEAR_RATIO);
    config.motionCruiseVelocity = SN_Math.metersToFalcon(prefElevator.elevatorMaxVelocity.getValue(),
        constElevator.CIRCUMFERENCE, constElevator.GEAR_RATIO);
    config.motionAcceleration = SN_Math.metersToFalcon(prefElevator.elevatorMaxAccel.getValue(),
        constElevator.CIRCUMFERENCE, constElevator.GEAR_RATIO);

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    config.forwardSoftLimitThreshold = SN_Math.metersToFalcon(prefElevator.elevatorMaxPos.getValue(),
        constElevator.CIRCUMFERENCE,
        constElevator.GEAR_RATIO);
    config.reverseSoftLimitThreshold = SN_Math.metersToFalcon(prefElevator.elevatorMinPos.getValue(),
        constElevator.CIRCUMFERENCE,
        constElevator.GEAR_RATIO);

    config.forwardSoftLimitEnable = true;
    config.reverseSoftLimitEnable = true;

    // https://v5.docs.ctr-electronics.com/en/stable/ch13_MC.html?highlight=Current%20limit#new-api-in-2020
    supplyLimit = new SupplyCurrentLimitConfiguration(true,
        constElevator.CURRENT_LIMIT_FLOOR_AMPS,
        constElevator.CURRENT_LIMIT_CEILING_AMPS,
        constElevator.CURRENT_LIMIT_AFTER_SEC);

    leftMotor.configAllSettings(config);
    rightMotor.configAllSettings(config);

    rightMotor.follow(leftMotor);
    leftMotor.setInverted(constElevator.INVERT_LEFT_MOTOR);
    rightMotor.setInverted(InvertType.OpposeMaster);

    rightMotor.configSupplyCurrentLimit(supplyLimit);
    leftMotor.configSupplyCurrentLimit(supplyLimit);
  }

  /**
   * Set the speed of the Elevator. Includes safeties/soft stops.
   * 
   * @param speed Desired speed to set both of the motors to, as a PercentOutput
   *              (-1.0 to 1.0)
   * 
   */
  public void setElevatorSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set the position of the Elevator. Includes safeties/soft stops.
   * 
   * @param position Desired position to set both of the motors to, in meters
   * 
   */
  public void setElevatorPosition(double position) {
    position = SN_Math.metersToFalcon(MathUtil.clamp(position,
        prefElevator.elevatorMinPos.getValue(),
        prefElevator.elevatorMaxPos.getValue()), constElevator.CIRCUMFERENCE, constElevator.GEAR_RATIO);

    desiredPosition = position;

    leftMotor.set(ControlMode.MotionMagic, position);
    rightMotor.set(ControlMode.MotionMagic, position);
  }

  /**
   * Returns if the elevator is within its positional tolerance.
   * 
   * @param desiredPosition Desired position, in meters
   * @param tolerance       The tolerance before we are considered at that
   *                        position, in meters
   * @return If it is at that position
   * 
   */
  public boolean isElevatorAtPosition(double desiredPosition, double tolerance) {
    if (Robot.isSimulation()) {
      return true;
    }
    return tolerance >= Math.abs(getElevatorPositionMeters() - desiredPosition);
  }

  /**
   * Returns the encoder counts of one motor on the elevator. (They should have
   * the same reading)
   * 
   * @return Elevator encoder counts
   * 
   */
  public double getElevatorEncoderCounts() {
    return rightMotor.getSelectedSensorPosition();
  }

  private double getElevatorAbsoluteEncoder() {
    double rotations = absoluteEncoder.get();
    rotations -= absoluteEncoderOffset;

    if (constElevator.ABSOLUTE_ENCODER_INVERT) {
      return -rotations;
    } else {
      return rotations;
    }
  }

  /**
   * Gets if the absolute encoder was plugged in on init. Returns true if it was
   * unplugged. This will also return true in simulation, because there is no
   * absolute encoder to get a value from.
   */
  public boolean getElevatorEncoderUnplugged() {
    return absoluteEncoder.get() == 0.0;
  }

  public void resetElevatorEncoderToAbsolute() {
    if (getElevatorEncoderUnplugged()) {
      resetElevatorToZero();
      return; // ENCODER UNPLUGGED!!!!!
    }
    rightMotor.setSelectedSensorPosition(
        SN_Math.degreesToFalcon(Units.rotationsToDegrees(getElevatorAbsoluteEncoder()),
            constElevator.GEAR_RATIO) * 2);
    leftMotor.setSelectedSensorPosition(
        SN_Math.degreesToFalcon(Units.rotationsToDegrees(getElevatorAbsoluteEncoder()),
            constElevator.GEAR_RATIO) * 2);
  }

  public void resetElevatorToZero() {
    rightMotor.setSelectedSensorPosition(0);
    leftMotor.setSelectedSensorPosition(0);
  }

  /**
   * Returns the position of the elevator, relative to itself, in meters.
   * 
   * @return Elevator position
   * 
   */
  public double getElevatorPositionMeters() {
    if (Robot.isSimulation()) {
      return desiredPosition / prefElevator.elevatorEncoderCountsPerMeter.getValue();
    }
    return getElevatorEncoderCounts() / prefElevator.elevatorEncoderCountsPerMeter.getValue();
  }

  public void neutralElevatorOutputs() {
    leftMotor.neutralOutput();
    rightMotor.neutralOutput();
  }

  public void setDesiredHeight(DesiredHeight height) {
    desiredHeight = height;
  }

  public DesiredHeight getDesiredHeight() {
    return desiredHeight;
  }

  public void setIsPrepped(boolean prepped) {
    isPrepped = prepped;
  }

  public boolean isPrepped() {
    return isPrepped;
  }

  // Must be run every loop in order for logging to function
  public void updatePose3ds() {
    elevatorCarriagePose = new Pose3d(
        -Math.cos(Math.toRadians(constElevator.ANGLE_TO_BASE_DEGREES)) * getElevatorPositionMeters(), 0,
        Math.sin(Math.toRadians(constElevator.ANGLE_TO_BASE_DEGREES)) * getElevatorPositionMeters(),
        new Rotation3d(0, 0, 0));
  }

  public Pose3d getElevatorCarriagePose() {
    return elevatorCarriagePose;
  }

  @Override
  public void periodic() {
    updatePose3ds();

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Encoder Counts Raw", getElevatorEncoderCounts());
    SmartDashboard.putNumber("Elevator/Position Meters", getElevatorPositionMeters());

    SmartDashboard.putNumber("Elevator/Abs Encoder Raw", absoluteEncoder.get());
    SmartDashboard.putNumber("Elevator/Abs Encoder Abs", absoluteEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Elevator/Abs Encoder Get", getElevatorAbsoluteEncoder());
    SmartDashboard.putBoolean("Elevator/Abs Encoder Unplugged?", getElevatorEncoderUnplugged());
    SmartDashboard.putNumber("Elevator/Supply Current", leftMotor.getSupplyCurrent());

    SmartDashboard.putNumberArray("Elevator/Stage 2 Pose3d", AdvantageScopeUtil.composePose3ds(elevatorStagePose));
    SmartDashboard.putNumberArray("Elevator/Carriage Pose3d",
        AdvantageScopeUtil.composePose3ds(getElevatorCarriagePose()));

  }
}
