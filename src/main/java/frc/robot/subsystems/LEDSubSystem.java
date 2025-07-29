// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LEDSubSystem extends SubsystemBase {
  /** Creates a new LEDSubSystem. */
  public static final double SOLID_GREEN = 0.77;
  public static final double SOLID_BlUE = 0.87;
  public static final double SOLID_RED = 0.61;
  public static final double SOLID_YELLOW = 0.69;
  public static final double SOLID_ORANGE = 0.65;
  public static final double PARTY_TWINKLE = -0.53;
  public static final double GOLD_STROBE = -0.07;

  Spark led;

  public LEDSubSystem() {
    led = new Spark(Constants.LEDSubSystemConstants.PWM_ID_LED);
  }

  public void setColor(double color){
    led.set(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
