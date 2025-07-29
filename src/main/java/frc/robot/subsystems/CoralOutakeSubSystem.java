// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralOutakeSubSystem extends SubsystemBase {
  /** Creates a new CoralOutakeSubSystem. */
  private TalonFX outtakeMotor;
  private CANrange sensor;

  public CoralOutakeSubSystem() {
    outtakeMotor = new TalonFX(Constants.CoralOutakeSubSystemConstants.CAN_ID_OUTTAKE);
    sensor = new CANrange(Constants.CoralOutakeSubSystemConstants.CAN_ID_SENSOR);
   /* ProximityParamsConfigs proxConfig = new ProximityParamsConfigs();
    proxConfig.ProximityThreshold = 0.16;//0.1525;
    proxConfig.ProximityHysteresis = 0.03;//0.0225;
    sensor.getConfigurator().apply(proxConfig);*/
    FovParamsConfigs fovConfig = new FovParamsConfigs();
    //fovConfig.withFOVRangeX(7);
    //fovConfig.withFOVRangeY(7);
    fovConfig.withFOVCenterX(11);
    sensor.getConfigurator().apply(fovConfig);
  }


  public void setSpeed(double speed){
    outtakeMotor.set(speed);
  }

  public boolean hasCoral(){
    return sensor.getDistance().getValueAsDouble() < 0.2;
  }
    

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Outtake Sensor", sensor.getDistance().getValueAsDouble());
    SmartDashboard.putBoolean("outtakeSensorTrigger", sensor.getIsDetected(true).getValue());
    //SmartDashboard.putNumber("strengthOfSignal", sensor.getSignalStrength().getValueAsDouble());
    /*double dis = sensor.getDistance(true).getValueAsDouble();
    boolean test = false; 
    if(dis < .18) {
      test=true;
    } else {
      test=false;
    } */
    //SmartDashboard.putBoolean("This is a boolean express because canrange sucks", test);
    //SmartDashboard.putNumber("Dis for Test", dis);
    // This method will be called once per scheduler run
  }
}
