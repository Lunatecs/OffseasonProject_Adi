// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimberSubSystem extends SubsystemBase {
  /** Creates a new ClimberSubSystem. */
  private TalonFX motor1;
  private TalonFX motor2;

  public ClimberSubSystem() {
    motor1 = new TalonFX(Constants.ClimberSubSystemConstants.CAN_ID_CLIMBER);
    motor1.setNeutralMode(NeutralModeValue.Brake);
    //motor2 = new TalonFX(Constants.ClimberSubSystemConstants.CAN_ID_CLIMBER);
    //motor2.setNeutralMode(NeutralModeValue.Brake);
    motor1.setPosition(0);
    //motor2.setControl(new Follower(motor1.getDeviceID(), true));
  }

  public void setSpeed(double speed) {
    motor1.set(speed);
  }

  public double getEncoder(){
    return motor1.getPosition().getValueAsDouble();
  }

  public double getCurrent() {
    return motor1.getStatorCurrent().getValueAsDouble();
  }

  

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climber encoder", getEncoder());
    SmartDashboard.putBoolean("DID WE CLIMB?", (getEncoder() <= Constants.ClimberSubSystemConstants.climbedSuccessfullyEncoderVal));
    SmartDashboard.putBoolean("BAD CLIMB", (getCurrent()>Constants.ClimberSubSystemConstants.climbCurrentLimit));
  }
}
