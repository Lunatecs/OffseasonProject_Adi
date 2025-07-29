// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class AlgaePivotSubSystem extends SubsystemBase {
  /** Creates a new AlgaePivotSubSystem. */
  private final double gearRatio = 62.5;//62.5/1
  private final double rotationsToDegrees = 360/gearRatio;
  private TalonFX pivotMotor;

  public AlgaePivotSubSystem() {
    pivotMotor = new TalonFX(Constants.AlgaePivotSubSystemConstants.CAN_ID_ALGAE_PIVOT);
    pivotMotor.setPosition(0);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.setInverted(true);
  }

  public void setSpeed(double speed){
    pivotMotor.set(speed);
  }

  public double getEncoder(){
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public double getDegreesOfPivot(){
    return getEncoder()* rotationsToDegrees;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Degrees", getDegreesOfPivot());
  }
}
