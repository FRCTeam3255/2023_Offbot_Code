// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotMap.mapLEDs;

public class LEDs extends SubsystemBase {

  CANdle CANdle;

  public LEDs() {
    CANdle = new CANdle(mapLEDs.CANDLE_CAN);
    configure();
  }

  public void configure() {
    CANdle.configFactoryDefault();
    CANdle.configBrightnessScalar(constLEDs.LED_BRIGHTNESS);
  }

  /**
   * Sets the LEDs to an array of RGB values
   *
   * @param rgb The desired array of RGB values. {r, g, b}
   */
  public void setLEDs(int[] rgb) {
    CANdle.setLEDs(rgb[0], rgb[1], rgb[2]);
  }

  /**
   * Sets the LEDs to an animation. Animation types can be found in
   * <a href=
   * "https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/led/Animation.html">CTRE's
   * javadocs.</a>
   *
   * 
   * @param animation The desired animation to display
   */
  public void setLEDsToAnimation(Animation animation) {
    CANdle.animate(animation, 0);
  }

  public void clearAnimation() {
    CANdle.clearAnimation(0);
  }

  @Override
  public void periodic() {
  }
}
