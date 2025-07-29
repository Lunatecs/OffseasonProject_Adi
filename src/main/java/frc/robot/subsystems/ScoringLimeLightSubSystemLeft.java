// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class ScoringLimeLightSubSystemLeft extends SubsystemBase {
  /** Creates a new ScoringLimeLightSubSystem. */
  private NetworkTable limelight;
  private NetworkTableEntry NetworkTx;
  private NetworkTableEntry NetworkTy;
  private NetworkTableEntry NetworkTa;
  private double[] botpose = new double[6];
  private String name = "limelight-left";
  
  public ScoringLimeLightSubSystemLeft() {
    limelight = NetworkTableInstance.getDefault().getTable(name);
    NetworkTx = limelight.getEntry("tx");
    NetworkTy = limelight.getEntry("ty");
    NetworkTa = limelight.getEntry("ta");
    LimelightHelpers.setPipelineIndex("limelight", 0);
  }

  @Override
  public void periodic() {
    
    botpose = LimelightHelpers.getBotPose_TargetSpace(name);
    if(botpose==null || botpose.length<6) {
      botpose = new double[6];
    }
    //SmartDashboard.putString("bot pose target LEFT", botpose[0] + " " + botpose[2]+ " " +  botpose[4]);
    SmartDashboard.putNumber("limelight LEFT x", getTranslationX());
    SmartDashboard.putNumber("limelight LEFT y", getTranslationY());

    //SmartDashboard.putString("values", poseA.getTranslation().getX() + " " + poseA.getTranslation().getY() + " " + poseA.getRotation());

    //this.botpose = NetworkBotPose.getDoubleArray(new double[8]);

    //SmartDashboard.putString("vision-left: x = ", this.GetTx() + ", y = " + this.GetTy());

    //SmartDashboard.putString("limelight vals", getTranslationX() + " " + getTranslationY() + " " + getYaw());
    //SmartDashboard.putNumber("t2d", getSkew());
  }

  public double getTranslationX() {
    return botpose[0];
  }

  public double getTranslationY() {
    return botpose[2];
  }


  public double getYaw() {
    return botpose[4];
  }

  public double GetTx(){
    return NetworkTx.getDouble(0.0);
    }
  
    public double GetTy(){
    return NetworkTy.getDouble(0.0);
    }
  
    public double GetTa(){
    return NetworkTa.getDouble(0.0);
    }
}
