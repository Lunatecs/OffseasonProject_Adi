// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.field;
import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class ReefPose {

    private Pose2d leftPose2d;
    private Pose2d rightPose2d;
    private double centerX;
    private double centerY;
    private int aprilTagNum;

    public ReefPose(int aprilTagNum, Pose2d rightPose, Pose2d leftPose, double centerX, double centerY) {
        this.aprilTagNum = aprilTagNum;
        this.leftPose2d = leftPose;
        this.rightPose2d = rightPose;
        this.centerX = centerX;
        this.centerY = centerY;
    }

    public int getAprilTagNumber(){
        return aprilTagNum;
    }

    public Pose2d getLeftPose(){
        return leftPose2d;
    }

    public Pose2d getRightPose(){
        return rightPose2d;
    }

    public double getCenterX(){
        return centerX;
    }

    public double getCenterY(){
        return centerY;
    }
}
