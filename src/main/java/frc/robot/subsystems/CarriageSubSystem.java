// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CarriageSubSystem extends SubsystemBase {
  private TalonFX carriageMotor;
  /** Creates a new CarriageSubSystem. */
  public CarriageSubSystem() {
    carriageMotor = new TalonFX(Constants.CarriageSubSystemConstants.CAN_ID_CARRIAGE);
    carriageMotor.setInverted(true);
  }

  public void setSpeed(double speed){
    carriageMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
