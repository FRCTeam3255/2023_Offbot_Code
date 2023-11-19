// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapLEDs;

public class LEDs extends SubsystemBase {

  SN_Blinkin ledController;
  CANdle candle;

  public LEDs() {
    ledController = new SN_Blinkin(mapLEDs.BLINKIN_PWM);
    candle = new CANdle(0);
  }

  public void setLEDs(int[] rgb) {
    candle.setLEDs(rgb[0], rgb[1], rgb[2]);
  }

  @Override
  public void periodic() {
  }
}
