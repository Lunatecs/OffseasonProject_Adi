// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.field;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Field {

    private static HashMap<Integer,ReefPose> blueNear = new HashMap<>();
    private static HashMap<Integer,ReefPose> blueNearNew = new HashMap<>();
    private static HashMap<Integer, ReefPose> blueNearNewNew = new HashMap<>();
    private static HashMap<Integer, ReefPose> blueNearFinal = new HashMap<>();
    private static HashMap<Integer, ReefPose> redNearFinal = new HashMap<>();
    private static HashMap<Integer, ReefPose> redNear = new HashMap<>();
    private static HashMap<Integer, ReefPose> blueFar = new HashMap<>();
    private static HashMap<Integer, ReefPose> redFar = new HashMap<>();

    private static Field  field = new Field();


    private Field(){

        blueNearNew.put(17, new ReefPose(17, 
            new Pose2d(4.08, 3.78, Rotation2d.fromDegrees(-120)), 
            new Pose2d(4.38, 3.61, Rotation2d.fromDegrees(-120)), 
            4.26, 3.67));

        blueNearNew.put(18, new ReefPose(18,
         new Pose2d(4.06, 4.27, Rotation2d.fromDegrees(180)),
          new Pose2d(4.07, 3.9, Rotation2d.fromDegrees(180)),
          4.07, 4.03));

        blueNearNew.put(19, new ReefPose(19,
         new Pose2d(4.49, 4.55, Rotation2d.fromDegrees(120)),
          new Pose2d(4.21, 4.34, Rotation2d.fromDegrees(120)),
           4.33, 4.42));

        blueNearNew.put(20, new ReefPose(20,
         new Pose2d(4.92, 4.29, Rotation2d.fromDegrees(-120)),
          new Pose2d(4.58, 4.44, Rotation2d.fromDegrees(-120)),
           4.73, 4.38));
          
           

        blueNearNew.put(21, new ReefPose(21,
         new Pose2d(4.91, 3.87, Rotation2d.fromDegrees(0)),
          new Pose2d(4.91, 4.14, Rotation2d.fromDegrees(0)),
           4.91, 4.02));

        blueNearNew.put(22, new ReefPose(22,
         new Pose2d(4.56, 3.58, Rotation2d.fromDegrees(-60)),
         new Pose2d(4.84, 3.73, Rotation2d.fromDegrees(-60)),
          4.67, 3.66));





        
          blueNearNewNew.put(17, new ReefPose(17,
          new Pose2d(3.995, 2.844, Rotation2d.fromDegrees(60)), 
          new Pose2d(3.714, 3.01, Rotation2d.fromDegrees(60)), 
          3.856, 2.927));
          
          blueNearNewNew.put(18, new ReefPose(18,
          new Pose2d(3.219, 3.86, Rotation2d.fromDegrees(0)), 
          new Pose2d(3.219, 4.185, Rotation2d.fromDegrees(0)), 
          3.219, 4.023)); 
          
          blueNearNewNew.put(19, new ReefPose(19,
          new Pose2d(3.713, 5.04, Rotation2d.fromDegrees(-60)), 
          new Pose2d(3.995, 5.193, Rotation2d.fromDegrees(-60)), 
          3.863, 5.121));

        blueNearNewNew.put(20, new ReefPose(20,
         new Pose2d(4.966, 5.197, Rotation2d.fromDegrees(-120)), 
         new Pose2d(5.265, 5.037, Rotation2d.fromDegrees(-120)), 
         5.105, 5.107));

         blueNearNewNew.put(21, new ReefPose(21,
         new Pose2d(5.754, 4.189, Rotation2d.fromDegrees(180)), 
         new Pose2d(5.754, 3.862, Rotation2d.fromDegrees(180)), 
         5.754, 4.021));
         
         blueNearNewNew.put(22, new ReefPose(22,
         new Pose2d(5.264, 3.008, Rotation2d.fromDegrees(120)), 
         new Pose2d(4.976, 2.852, Rotation2d.fromDegrees(120)), 
         5.12, 2.928));










         blueNearFinal.put(17, new ReefPose(17,
         new Pose2d(4.007, 2.86, Rotation2d.fromDegrees(60)), 
         new Pose2d(3.721, 3.026, Rotation2d.fromDegrees(60)), 
         4.07, 3.306));
         
         blueNearFinal.put(18, new ReefPose(18,
         new Pose2d(3.239, 3.86, Rotation2d.fromDegrees(0)), 
         new Pose2d(3.239, 4.191, Rotation2d.fromDegrees(0)), 
         3.658, 4.026)); 
         
         blueNearFinal.put(19, new ReefPose(19,
         new Pose2d(3.721, 5.026, Rotation2d.fromDegrees(-60)), 
         new Pose2d(4.007, 5.19, Rotation2d.fromDegrees(-60)), 
         4.074, 4.745));

       blueNearFinal.put(20, new ReefPose(20,
        new Pose2d(4.971, 5.19, Rotation2d.fromDegrees(-120)), 
        new Pose2d(5.257, 5.026, Rotation2d.fromDegrees(-120)), 
        4.905, 4.745));

        blueNearFinal.put(21, new ReefPose(21,
        new Pose2d(5.74, 4.191, Rotation2d.fromDegrees(180)), 
        new Pose2d(5.73, 3.86, Rotation2d.fromDegrees(180)), 
        5.321, 4.026));
        
        blueNearFinal.put(22, new ReefPose(22,
        new Pose2d(5.257, 3.026, Rotation2d.fromDegrees(120)), 
        new Pose2d(4.971, 2.86, Rotation2d.fromDegrees(120)), 
        4.90, 3.306));



        redNearFinal.put(6, new ReefPose(6,
         new Pose2d(13.827, 3.0259, Rotation2d.fromDegrees(120)),
          new Pose2d(13.541, 2.8608, Rotation2d.fromDegrees(120)),
           13.4744, 3.3063));

         redNearFinal.put(7, new ReefPose(7,
          new Pose2d(14.3096, 4.191, Rotation2d.fromDegrees(180)),
           new Pose2d(14.3096, 3.8608, Rotation2d.fromDegrees(180)),
            13.8905, 4.0295));

         redNearFinal.put(8, new ReefPose(8,
          new Pose2d(13.541, 5.191, Rotation2d.fromDegrees(-120)),
           new Pose2d(13.827, 5.0259, Rotation2d.fromDegrees(-120)),
            13.4744, 4.7455));

         redNearFinal.put(9, new ReefPose(9,
          new Pose2d(12.2908, 5.0259, Rotation2d.fromDegrees(-60)),
           new Pose2d(12.5768, 5.191, Rotation2d.fromDegrees(-60)),
            12.6434, 4.7455));

         redNearFinal.put(10, new ReefPose(10,
          new Pose2d(11.8082, 3.8608, Rotation2d.fromDegrees(0)),
           new Pose2d(11.8082, 4.191, Rotation2d.fromDegrees(0)),
            12.2273, 4.0259));

         redNearFinal.put(11, new ReefPose(11,
          new Pose2d(12.5768, 2.8608, Rotation2d.fromDegrees(60)),
           new Pose2d(12.2908, 3.0259, Rotation2d.fromDegrees(60)),
            12.6434, 3.3063));

        



        blueNear.put(17, new ReefPose(17, 
            new Pose2d(3.994, 2.83, Rotation2d.fromDegrees(60)), 
            new Pose2d(3.709, 3.007, Rotation2d.fromDegrees(60)), 
            3.85, 2.923));

        blueNear.put(18, new ReefPose(18,
         new Pose2d(3.2, 3.865, Rotation2d.fromDegrees(0)),
          new Pose2d(3.2, 4.193, Rotation2d.fromDegrees(0)),
          3.227, 4.025));

        blueNear.put(19, new ReefPose(19,
         new Pose2d(3.71, 5.044, Rotation2d.fromDegrees(-60)),
          new Pose2d(3.994, 5.21, Rotation2d.fromDegrees(-60)),
           3.846, 5.12));

        blueNear.put(20, new ReefPose(20,
         new Pose2d(4.984, 5.209, Rotation2d.fromDegrees(-120)),
          new Pose2d(5.266, 5.044, Rotation2d.fromDegrees(-120)),
           5.128, 5.122));

        blueNear.put(21, new ReefPose(21,
         new Pose2d(5.763, 4.192, Rotation2d.fromDegrees(180)),
          new Pose2d(5.763, 3.863, Rotation2d.fromDegrees(180)),
           5.757, 4.017));

        blueNear.put(22, new ReefPose(22,
         new Pose2d(5.264, 3.004, Rotation2d.fromDegrees(120)),
         new Pose2d(4.981, 2.841, Rotation2d.fromDegrees(120)),
          5.125, 2.924));

        redNear.put(6, new ReefPose(6,
         new Pose2d(13.842, 3.005, Rotation2d.fromDegrees(120)),
          new Pose2d(13.558, 2.84, Rotation2d.fromDegrees(120)),
           13.703, 2.925));

        redNear.put(7, new ReefPose(7,
         new Pose2d(14.38, 4.192, Rotation2d.fromDegrees(180)),
          new Pose2d(14.38, 3.865, Rotation2d.fromDegrees(180)),
           14.361, 4.04));

        redNear.put(8, new ReefPose(8,
         new Pose2d(13.579, 5.248, Rotation2d.fromDegrees(-120)),
          new Pose2d(13.863, 5.083, Rotation2d.fromDegrees(-120)),
           13.72, 5.167));

        redNear.put(9, new ReefPose(9,
         new Pose2d(12.261, 5.083, Rotation2d.fromDegrees(-60)),
          new Pose2d(12.546, 5.251, Rotation2d.fromDegrees(-60)),
           12.402, 5.166));

        redNear.put(10, new ReefPose(10,
        new Pose2d(11.749, 3.865, Rotation2d.fromDegrees(0)),
        new Pose2d(11.749, 4.192, Rotation2d.fromDegrees(0)),
           11.749, 4.015));

        redNear.put(11, new ReefPose(11,
         new Pose2d(12.571, 2.841, Rotation2d.fromDegrees(60)),
        new Pose2d(12.284, 3.007, Rotation2d.fromDegrees(60)),
           12.433, 2.918));


        blueFar.put(17, new ReefPose(17, 
           new Pose2d(3.964, 2.789, Rotation2d.fromDegrees(60)), 
           new Pose2d(2.789, 3.682, Rotation2d.fromDegrees(60)), 
           3.85, 2.923));

       blueFar.put(18, new ReefPose(18,
        new Pose2d(3.175, 3.865, Rotation2d.fromDegrees(0)),
         new Pose2d(3.175, 4.193, Rotation2d.fromDegrees(0)),
         3.227, 4.025));

       blueFar.put(19, new ReefPose(19,
        new Pose2d(3.687, 5.081, Rotation2d.fromDegrees(-60)),
         new Pose2d(3.969, 5.258, Rotation2d.fromDegrees(-60)),
          3.846, 5.12));

       blueFar.put(20, new ReefPose(20,
        new Pose2d(5.01, 5.26, Rotation2d.fromDegrees(-120)),
         new Pose2d(5.293, 5.089, Rotation2d.fromDegrees(-120)),
          5.128, 5.122));

       blueFar.put(21, new ReefPose(21,
        new Pose2d(5.814, 4.912, Rotation2d.fromDegrees(180)),
         new Pose2d(5.809, 3.858, Rotation2d.fromDegrees(180)),
          5.757, 4.017));

       blueFar.put(22, new ReefPose(22,
        new Pose2d(5.295, 2.957, Rotation2d.fromDegrees(120)),
        new Pose2d(5.005, 2.802, Rotation2d.fromDegrees(120)),
         5.125, 2.924));

       redFar.put(6, new ReefPose(6,
        new Pose2d(13.862, 2.97, Rotation2d.fromDegrees(120)),
         new Pose2d(13.578, 2.805, Rotation2d.fromDegrees(120)),
          13.703, 2.925));

       redFar.put(7, new ReefPose(7,
        new Pose2d(14.405, 4.192, Rotation2d.fromDegrees(180)),
         new Pose2d(14.405, 3.865, Rotation2d.fromDegrees(180)),
          14.361, 4.04));

       redFar.put(8, new ReefPose(8,
        new Pose2d(13.59, 5.262, Rotation2d.fromDegrees(-120)),
         new Pose2d(13.877, 5.107, Rotation2d.fromDegrees(-120)),
          13.72, 5.167));

       redFar.put(9, new ReefPose(9,
        new Pose2d(12.249, 5.107, Rotation2d.fromDegrees(-60)),
         new Pose2d(12.534, 5.272, Rotation2d.fromDegrees(-60)),
          12.402, 5.166));

       redFar.put(10, new ReefPose(10,
       new Pose2d(11.724, 3.865, Rotation2d.fromDegrees(0)),
       new Pose2d(11.724, 4.192, Rotation2d.fromDegrees(0)),
          11.749, 4.015));

       redFar.put(11, new ReefPose(11,
        new Pose2d(12.532, 2.786, Rotation2d.fromDegrees(60)),
       new Pose2d(12.255, 2.958, Rotation2d.fromDegrees(60)),
          12.433, 2.918));
    }

    public static Field getField() {
        return field;
    }

    public ReefPose getClosestReefPose(Pose2d robotPose) {
       // return Field.blueNearNewNew.get(20);
         
        Alliance color = DriverStation.getAlliance().orElse(Alliance.Blue);
        HashMap<Integer,ReefPose> reefMap = null;
        if(color.equals(Alliance.Blue)) {
            reefMap = Field.blueNearFinal;
        } else {
            reefMap = Field.redNearFinal; //Field.redNear;
        }

        Set<Entry<Integer,ReefPose>> values = reefMap.entrySet();

        Iterator<Entry<Integer,ReefPose>> iter = values.iterator();

        double shortest = 1000;
        int aprilTagNum = -1;
        String lengths = "";
        while(iter.hasNext()) {
            Entry<Integer,ReefPose> value = iter.next();
            double length = getLineLength(value.getValue().getCenterX(), robotPose.getTranslation().getX(), 
                                            value.getValue().getCenterY(), robotPose.getTranslation().getY());
            lengths = value.getValue().getAprilTagNumber() + ": " + length + "\r\n" + lengths;
            SmartDashboard.putString("April Tag " + value.getValue().getAprilTagNumber(), 
                    length + "= " +
                    value.getValue().getCenterX() + "-" + robotPose.getTranslation().getX() + "+" + 
                    value.getValue().getCenterY() + "-" + robotPose.getTranslation().getY());
            if(length < shortest){
                shortest = length;
                aprilTagNum = value.getValue().getAprilTagNumber();
            }

        }
        
        SmartDashboard.putString("lengths", lengths);

        return reefMap.get(aprilTagNum);
    }

    public double getLineLength(double x1, double x2, double y1, double y2){
        double lineLength = Math.sqrt(Math.pow((x1 - x2), 2) + Math.pow((y1 - y2), 2));
        return lineLength;
    }

}
