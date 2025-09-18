// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static double DEFAULT_ALGEA_INTAKE = .1;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CoralGroundIntakeSubSystemConstants{
    public static final int CAN_ID_INTAKE = 13;
  }

  public static class CarriageSubSystemConstants {
    public static final int CAN_ID_CARRIAGE =11;
  }

  public final class CoralGroundIntakePivotSubSystemConstants{
    public static final int CAN_ID_INTAKE_PIVOT = 12;
  }

  public static class CoralOutakeSubSystemConstants{
    public static final int CAN_ID_OUTTAKE = 15;
    public static final int CAN_ID_SENSOR = 22;
  }

  public static class CoralAlignmentSubSystemConstants{
    public static final int CAN_ID_SENSOR2 = 21;
  }

  public static class ClimberSubSystemConstants{
    public static final int CAN_ID_CLIMBER = 20;
    public static final double climbedSuccessfullyEncoderVal = -60; //needs to be tested
    public static final double climbCurrentLimit = 150.0; //needs to be tested
    
    //switched from 17 to 20
  }

  public static class ElevatorSubSystemConstants{
    public static final int CAN_ID_MOTOR1 = 9;
    public static final int CAN_ID_MOTOR2 = 10;

    public static final int INPUT_ID = 0;
  } 

  public static class CoralFeederSubSystemConstants{
    public static final int CAN_ID_FEEDER = 14;
  }

  //public static class CoralHopperSubSystemConstants{
    //public static final int CAN_ID_HOPPER = 20;
    //Switched from 20 to 17, because the hopper motor is the climber
 // }

  public static class AlgaePivotSubSystemConstants{
    public static final int CAN_ID_ALGAE_PIVOT = 19;
  }

  public static class AlgaeLiberatorSubSystemConstants{
    public static final int CAN_ID_ALGAE_LIBERATOR = 16;
  }

  public static class LEDSubSystemConstants{
    public static final int PWM_ID_LED = 9;
  }
  public class DrivetrainConstants {
    public static final double kMaxSpeed = 5.41;
    public static final double kMaxAngularRate = kMaxSpeed * 39.37 / 20.75 * Math.PI;
}
  public class OperatorrConstants {
    public static final int kL4 = 8;
    public static final int kL3 = 9;
    public static final int kL2 = 11;
    public static final int kL1 = 6;
    public static final int kAutoAlignLeft = 10;
    public static final int kAutoAlignRight = 2;
    public static final int kAL3 = 7;
    public static final int kAL2 = 12;
    public static final int kNET = 4;
    public static final int kProcs = 3;
    public static final int kT = 5;
  }
  public class ReefPoses {
    public static final Pose2d kRED0_1 = new Pose2d(11.19, 4.25, Rotation2d.fromDegrees(0));
    public static final Pose2d kRED2_3 = new Pose2d(11.92 , 6.41, Rotation2d.fromDegrees(-60));
    public static final Pose2d kRED4_5 = new Pose2d(14.46, 6.51, Rotation2d.fromDegrees(-120));
    public static final Pose2d kRED6_7 = new Pose2d(15.47, 4, Rotation2d.fromDegrees(180));
    public static final Pose2d kRED8_9 = new Pose2d(14.44, 1.67, Rotation2d.fromDegrees(120));
    public static final Pose2d kRED10_11 = new Pose2d(11.99, 1.85, Rotation2d.fromDegrees(60));
    public static final Pose2d kRED10_11_ALGAE = new Pose2d(12.257259368896484, 2.6072371006011963, Rotation2d.fromDegrees(60));
    public static final Pose2d kREDBarge = new Pose2d(10.1, 3.2843942642211914, Rotation2d.fromDegrees(-180));

    public static final Pose2d kREDSOURCERIGHT_center = new Pose2d(16.42, 7.06, Rotation2d.fromDegrees(-128));
    public static final Pose2d kREDSOURCERIGHT_bargeWall = new Pose2d(15.93, 7.46, Rotation2d.fromDegrees(-128));
    public static final Pose2d kREDSOURCERIGHT_operatorWall_shifted = new Pose2d(16.50611686706543, 6.901570796966553, Rotation2d.fromDegrees(-128));
    public static final Pose2d kREDSOURCERIGHT_operatorWall = new Pose2d(16.87, 6.516, Rotation2d.fromDegrees(-128));
    
    public static final Pose2d kREDSOURCELEFT_center = new Pose2d(16.42, 0.97, Rotation2d.fromDegrees(128));
    public static final Pose2d kREDSOURCELEFT_bargeWall = new Pose2d(15.86, 0.55, Rotation2d.fromDegrees(128));
    public static final Pose2d kREDSOURCELEFT_operatorWall = new Pose2d(16.61, 1.3, Rotation2d.fromDegrees(128));

    public static final Pose2d kBLUE0_1 = new Pose2d(6.35, 4.1, Rotation2d.fromDegrees(180)); 
    public static final Pose2d kBLUE2_3 = new Pose2d(5.68, 1.72, Rotation2d.fromDegrees(120));
    public static final Pose2d kBLUE4_5 = new Pose2d(3.1, 1.75, Rotation2d.fromDegrees(60));
    public static final Pose2d kBLUE6_7 = new Pose2d(2.1, 4.08, Rotation2d.fromDegrees(0));
    public static final Pose2d kBLUE8_9 = new Pose2d(3.24, 6.34, Rotation2d.fromDegrees(-60));
    public static final Pose2d kBLUE10_11 = new Pose2d(5.49, 6.29, Rotation2d.fromDegrees(-120));
    public static final Pose2d kBLUE10_11_ALGAE = new Pose2d(5.3, 5.61, Rotation2d.fromDegrees(-120));
    public static final Pose2d kBLUEBarge = new Pose2d(7.259877681732178, 5.142665386199951, Rotation2d.fromDegrees(0));
    
    public static final Pose2d kBLUESOURCERIGHT_center = new Pose2d(1.12, 0.93, Rotation2d.fromDegrees(52));
    public static final Pose2d kBLUESOURCERIGHT_bargeWall = new Pose2d(1.57, 0.54, Rotation2d.fromDegrees(52));
    public static final Pose2d kBLUESOURCERIGHT_operatorWall = new Pose2d(0.87, 1.32, Rotation2d.fromDegrees(52));

    public static final Pose2d kBLUESOURCELEFT_center = new Pose2d(1.29, 7.17, Rotation2d.fromDegrees(-52));
    public static final Pose2d kBLUESOURCELEFT_operatorWall = new Pose2d(0.85, 6.67, Rotation2d.fromDegrees(-52));
    public static final Pose2d kBLUESOURCELEFT_bargeWall = new Pose2d(1.64, 7.42, Rotation2d.fromDegrees(-52));

    public static final PathConstraints K_CONSTRAINTS_Fastest = new PathConstraints(5.41, 6., 3*Math.PI, 3*Math.PI);
    public static final PathConstraints K_CONSTRAINTS_Barging = new PathConstraints(3, 4., 3*Math.PI, 3*Math.PI);
  }

public class NotOurAlignmentPoses {
    public static final Pose2d[] kAliRED0_1 = new Pose2d[]{new Pose2d(11.65, 3.77, Rotation2d.fromDegrees(0)), new Pose2d(11.65, 4.23, Rotation2d.fromDegrees(0))};
    public static final Pose2d[] kAliRED2_3 = new Pose2d[]{new Pose2d(12.42, 5.43, Rotation2d.fromDegrees(-60)), new Pose2d(12.09, 5.08, Rotation2d.fromDegrees(-60))};
    public static final Pose2d[] kAliRED4_5 = new Pose2d[]{new Pose2d(13.56, 5.37, Rotation2d.fromDegrees(-120)), new Pose2d(13.96, 5.14, Rotation2d.fromDegrees(-120))};
    public static final Pose2d[] kAliRED6_7 = new Pose2d[]{new Pose2d(14.49, 4.25, Rotation2d.fromDegrees(180)), new Pose2d(14.49, 3.86, Rotation2d.fromDegrees(180))};
    public static final Pose2d[] kAliRED8_9 = new Pose2d[]{new Pose2d(13.70, 2.57, Rotation2d.fromDegrees(60)), new Pose2d(13.99, 2.74, Rotation2d.fromDegrees(60))};
    public static final Pose2d[] kAliRED10_11 = new Pose2d[]{new Pose2d(12.16, 2.72, Rotation2d.fromDegrees(120)), new Pose2d(12.46, 2.55, Rotation2d.fromDegrees(120))};

    public static final Pose2d[] kAliBLUE6_7 = new Pose2d[]{new Pose2d(2.91, 3.85, Rotation2d.fromDegrees(0)), new Pose2d(3.2, 3.86, Rotation2d.fromDegrees(0))};
    public static final Pose2d[] kAliBLUE2_3 = new Pose2d[]{new Pose2d(5.11, 2.56, Rotation2d.fromDegrees(120)), new Pose2d(5.4, 2.76, Rotation2d.fromDegrees(120))};
    public static final Pose2d[] kAliBLUE4_5 = new Pose2d[]{new Pose2d(3.56, 2.74, Rotation2d.fromDegrees(60)), new Pose2d(3.86, 2.58, Rotation2d.fromDegrees(60))};
    public static final Pose2d[] kAliBLUE0_1 = new Pose2d[]{new Pose2d(6.07, 3.85, Rotation2d.fromDegrees(180)), new Pose2d(6.07, 4.18, Rotation2d.fromDegrees(180))};
    public static final Pose2d[] kAliBLUE8_9 = new Pose2d[]{new Pose2d(3.83, 5.47, Rotation2d.fromDegrees(-60)), new Pose2d(3.57, 5.33, Rotation2d.fromDegrees(-60))};
    public static final Pose2d[] kAliBLUE10_11 = new Pose2d[]{new Pose2d(5.43, 5.31, Rotation2d.fromDegrees(-120)), new Pose2d(5.14, 5.47, Rotation2d.fromDegrees(-120))};
  }
  public class NormalAlignmentPoses {
    public static final Pose2d[] kAliRED6 = new Pose2d[]{new Pose2d(13.827, 3.0259, Rotation2d.fromDegrees(120)), new Pose2d(13.541, 2.8608, Rotation2d.fromDegrees(120))};
    public static final Pose2d[] kAliRED7 = new Pose2d[]{new Pose2d(14.3096, 4.191, Rotation2d.fromDegrees(180)), new Pose2d(14.3096, 3.8608, Rotation2d.fromDegrees(180))};
    public static final Pose2d[] kAliRED8 = new Pose2d[]{new Pose2d(13.541, 5.191, Rotation2d.fromDegrees(-120)), new Pose2d(13.827, 5.0259, Rotation2d.fromDegrees(-120))};
    public static final Pose2d[] kAliRED9 = new Pose2d[]{new Pose2d(12.2908, 5.0259, Rotation2d.fromDegrees(-60)), new Pose2d(12.5768, 5.191, Rotation2d.fromDegrees(-60))};
    public static final Pose2d[] kAliRED10 = new Pose2d[]{new Pose2d(11.8082, 3.8608, Rotation2d.fromDegrees(0)), new Pose2d(11.8082, 4.191, Rotation2d.fromDegrees(0))};
    public static final Pose2d[] kAliRED11 = new Pose2d[]{new Pose2d(12.5768, 2.8608, Rotation2d.fromDegrees(60)), new Pose2d(12.2908, 3.0259, Rotation2d.fromDegrees(60))};

    public static final Pose2d[] kAliBLUE17 = new Pose2d[]{new Pose2d(4.007, 2.86, Rotation2d.fromDegrees(60)), new Pose2d(3.721, 3.026, Rotation2d.fromDegrees(60)), };
    public static final Pose2d[] kAliBLUE18 = new Pose2d[]{new Pose2d(3.239, 3.86, Rotation2d.fromDegrees(0)), new Pose2d(3.239, 4.191, Rotation2d.fromDegrees(0))};
    public static final Pose2d[] kAliBLUE19 = new Pose2d[]{new Pose2d(3.721, 5.026, Rotation2d.fromDegrees(-60)), new Pose2d(4.007, 5.19, Rotation2d.fromDegrees(-60))};
    public static final Pose2d[] kAliBLUE20 = new Pose2d[]{new Pose2d(4.971, 5.19, Rotation2d.fromDegrees(-120)), new Pose2d(5.257, 5.026, Rotation2d.fromDegrees(-120))};
    public static final Pose2d[] kAliBLUE21 = new Pose2d[]{new Pose2d(5.74, 4.191, Rotation2d.fromDegrees(180)), new Pose2d(5.73, 3.86, Rotation2d.fromDegrees(180))};
    public static final Pose2d[] kAliBLUE22 = new Pose2d[]{new Pose2d(5.257, 3.026, Rotation2d.fromDegrees(120)), new Pose2d(4.971, 2.86, Rotation2d.fromDegrees(120))};
  }
  public class RaritanAlignmentPoses { //Adi Method
    public static final Pose2d[] kAliRED6 = new Pose2d[]{new Pose2d(13.5032, 3.0259, Rotation2d.fromDegrees(120)), new Pose2d(13.2172, 2.8608, Rotation2d.fromDegrees(120))};
    public static final Pose2d[] kAliRED7 = new Pose2d[]{new Pose2d(13.9858, 4.191, Rotation2d.fromDegrees(180)), new Pose2d(13.9858, 3.8608, Rotation2d.fromDegrees(180))};
    public static final Pose2d[] kAliRED8 = new Pose2d[]{new Pose2d(13.2172, 5.191, Rotation2d.fromDegrees(-120)), new Pose2d(13.5032, 5.0259, Rotation2d.fromDegrees(-120))};
    public static final Pose2d[] kAliRED9 = new Pose2d[]{new Pose2d(11.967, 5.0259, Rotation2d.fromDegrees(-60)), new Pose2d(12.253, 5.191, Rotation2d.fromDegrees(-60))};
    public static final Pose2d[] kAliRED10 = new Pose2d[]{new Pose2d(11.4844, 3.8608, Rotation2d.fromDegrees(0)), new Pose2d(11.4844, 4.191, Rotation2d.fromDegrees(0))};
    public static final Pose2d[] kAliRED11 = new Pose2d[]{new Pose2d(12.253, 2.8608, Rotation2d.fromDegrees(60)), new Pose2d(11.967, 3.0259, Rotation2d.fromDegrees(60))};

    public static final Pose2d[] kAliBLUE17 = new Pose2d[]{new Pose2d(4.3308, 2.86, Rotation2d.fromDegrees(60)), new Pose2d(4.0448, 3.026, Rotation2d.fromDegrees(60)), };
    public static final Pose2d[] kAliBLUE18 = new Pose2d[]{new Pose2d(3.5628, 3.86, Rotation2d.fromDegrees(0)), new Pose2d(3.5628, 4.191, Rotation2d.fromDegrees(0))};
    public static final Pose2d[] kAliBLUE19 = new Pose2d[]{new Pose2d(4.0448, 5.026, Rotation2d.fromDegrees(-60)), new Pose2d(4.3308, 5.19, Rotation2d.fromDegrees(-60))};
    public static final Pose2d[] kAliBLUE20 = new Pose2d[]{new Pose2d(5.2948, 5.19, Rotation2d.fromDegrees(-120)), new Pose2d(5.5808, 5.026, Rotation2d.fromDegrees(-120))};
    public static final Pose2d[] kAliBLUE21 = new Pose2d[]{new Pose2d(6.0638, 4.191, Rotation2d.fromDegrees(180)), new Pose2d(6.0538, 3.86, Rotation2d.fromDegrees(180))};
    public static final Pose2d[] kAliBLUE22 = new Pose2d[]{new Pose2d(5.5808, 3.026, Rotation2d.fromDegrees(120)), new Pose2d(5.2948, 2.86, Rotation2d.fromDegrees(120))};
  }
  public class RoboconRealMapAlignmentposes { // Actual Field Method

    // RED Alliance 6-7???
    public static final Pose2d[] kAliRED6 = new Pose2d[]{
        new Pose2d(12.56105, 3.03555, Rotation2d.fromDegrees(120)), 
        new Pose2d(12.27965, 2.87305, Rotation2d.fromDegrees(120))
    };
    public static final Pose2d[] kAliRED7 = new Pose2d[]{
        new Pose2d(13.0397, 4.1884, Rotation2d.fromDegrees(180)), 
        new Pose2d(13.0397, 3.8634, Rotation2d.fromDegrees(180))
    };
    public static final Pose2d[] kAliRED8 = new Pose2d[]{
        new Pose2d(12.27965, 5.17875, Rotation2d.fromDegrees(-120)), 
        new Pose2d(12.56105, 5.01625, Rotation2d.fromDegrees(-120))
    };
    public static final Pose2d[] kAliRED9 = new Pose2d[]{
        new Pose2d(11.04215, 5.01625, Rotation2d.fromDegrees(-60)), 
        new Pose2d(11.32355, 5.17875, Rotation2d.fromDegrees(-60))
    };
    public static final Pose2d[] kAliRED10 = new Pose2d[]{
        new Pose2d(10.5635, 3.8634, Rotation2d.fromDegrees(0)), 
        new Pose2d(10.5635, 4.1884, Rotation2d.fromDegrees(0))
    };
    public static final Pose2d[] kAliRED11 = new Pose2d[]{
        new Pose2d(11.32355, 2.87305, Rotation2d.fromDegrees(60)), 
        new Pose2d(11.04215, 3.03555, Rotation2d.fromDegrees(60))
    };

    // BLUE Alliance (tags 17–22)
    public static final Pose2d[] kAliBLUE17 = new Pose2d[]{
        new Pose2d(3.40175, 2.87305, Rotation2d.fromDegrees(60)), 
        new Pose2d(3.12035, 3.03555, Rotation2d.fromDegrees(60))
    };
    public static final Pose2d[] kAliBLUE18 = new Pose2d[]{
        new Pose2d(2.6415, 3.8634, Rotation2d.fromDegrees(0)), 
        new Pose2d(2.6415, 4.1884, Rotation2d.fromDegrees(0))
    };
    public static final Pose2d[] kAliBLUE19 = new Pose2d[]{
        new Pose2d(3.12035, 5.01625, Rotation2d.fromDegrees(-60)), 
        new Pose2d(3.4018, 5.1788, Rotation2d.fromDegrees(-60))
    };
    public static final Pose2d[] kAliBLUE20 = new Pose2d[]{
        new Pose2d(4.35765, 5.17875, Rotation2d.fromDegrees(-120)), 
        new Pose2d(4.63905, 5.01625, Rotation2d.fromDegrees(-120))
    };
    public static final Pose2d[] kAliBLUE21 = new Pose2d[]{
        new Pose2d(5.1179, 3.8634, Rotation2d.fromDegrees(-180)), 
        new Pose2d(5.1179, 4.1884, Rotation2d.fromDegrees(-180))
    };
    public static final Pose2d[] kAliBLUE22 = new Pose2d[]{
        new Pose2d(4.63905, 3.03555, Rotation2d.fromDegrees(120)), 
        new Pose2d(4.35765, 2.87305, Rotation2d.fromDegrees(120))
    };
}

}
