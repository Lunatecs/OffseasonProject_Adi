// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChuteReleaseSubSystem extends SubsystemBase {
  /** Creates a new WhaleTailReleaseSubSystem. */
  private final Servo dropServo = new Servo(8);  // CHANGED

  public ChuteReleaseSubSystem() {
    dropServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);    
    //setBounds(2.0,1.8,1.5,1.2,1.0);
    dropServo.set(1.0);
  }

  public void releaseChute() {
      dropServo.set(0.0);
  }

  public void setChute(){
    dropServo.set(1);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("servo value", dropServo.get());
  }
}
