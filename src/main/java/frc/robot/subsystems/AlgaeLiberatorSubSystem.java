// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;


public class AlgaeLiberatorSubSystem extends SubsystemBase {
  /** Creates a new AlgaeLiberatorSubSystem. */

  public TalonFX intakeMotor;

  public AlgaeLiberatorSubSystem() {
    
    TalonFXConfiguration config = new TalonFXConfiguration().withCurrentLimits( new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(40)).withStatorCurrentLimitEnable(true));
    intakeMotor = new TalonFX(Constants.AlgaeLiberatorSubSystemConstants.CAN_ID_ALGAE_LIBERATOR);
    intakeMotor.getConfigurator().apply(config);
  }

  public void setSpeed(double speed){
    intakeMotor.set(speed);
  }

  public double getMotorCurrent(){
    return intakeMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Current", intakeMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Algae Volts", intakeMotor.getMotorVoltage().getValueAsDouble());
  }
}
