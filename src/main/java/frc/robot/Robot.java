// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.hardware.CANrange;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot  {
  private Command m_autonomousCommand;
  private Rotation2d autonEndingAngle = new Rotation2d();
  private final CommandSwerveDrivetrain swerve;


  private final RobotContainer m_robotContainer;

  //private final CANrange sensor = new CANrange(62);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  public Robot() {
    //FlippingUtil.fieldSizeX = 15.68;
    //LINE ABOVE ONLY ACTIVATED IF USING REALROBOCON FIELD
    Logger.addDataReceiver(new NT4Publisher());


    m_robotContainer = new RobotContainer();   
    swerve = m_robotContainer.drivetrain; ; 

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    FollowPathCommand.warmupCommand().schedule();
    CommandScheduler.getInstance().schedule(
    Commands.defer(
        () -> AutoBuilder.pathfindToPose(m_robotContainer.drivetrain.getPose(), new PathConstraints(3, 4., 3*Math.PI, 3*Math.PI)),
        Set.of() // required subsystem dependencies if any
    )
    );
    Logger.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void driverStationConnected() {
      double targetRotationDegrees = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 0 : 180; //Switched 180 and 0, 4/1 SWITCHED IT BACK
      m_robotContainer.drivetrain.resetRotation(Rotation2d.fromDegrees(targetRotationDegrees));
  }

  @Override
  public void robotPeriodic() {

        // Run scheduler as usual
        CommandScheduler.getInstance().run();
    //SmartDashboard.putNumber("67", sensor.getDistance().getValueAsDouble());
    //SmartDashboard.putBoolean("detetction", sensor.getIsDetected().getValue().booleanValue());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    swerve.resetPoseBasedOnLL();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    autonEndingAngle = m_robotContainer.drivetrain.getState().Pose.getRotation();
    }

  @Override
  public void teleopInit() {
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.drivetrain.resetRotation(autonEndingAngle);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}