// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralAlignmentSubSystem extends SubsystemBase {
  /** Creates a new CoralOutakeSubSystem. */
  private CANrange sensor;

  public CoralAlignmentSubSystem() {
    sensor = new CANrange(Constants.CoralAlignmentSubSystemConstants.CAN_ID_SENSOR2);
    //ProximityParamsConfigs proxConfig = new ProximityParamsConfigs();
    //proxConfig.ProximityThreshold = 0.16;//0.1525;
    //proxConfig.ProximityHysteresis = 0.03;//0.0225;
    //sensor.getConfigurator().apply(proxConfig); 
    FovParamsConfigs fovConfig = new FovParamsConfigs();
    fovConfig.withFOVRangeX(7);
    fovConfig.withFOVRangeY(7);
    sensor.getConfigurator().apply(fovConfig);
  }


  public boolean isAlignedL4(){
    return sensor.getDistance().getValueAsDouble() < 0.35;
  }

  public boolean isAlignedL3() {
    return sensor.getDistance().getValueAsDouble() < 0.60;
  }

  public boolean isAlignedL2() {
    return sensor.getDistance().getValueAsDouble() < 0.60;
  }
    

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Alignment Sensor", sensor.getDistance().getValueAsDouble());
    SmartDashboard.putBoolean("You're clear kid.", isAlignedL4());
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
