// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//hi jugad
package frc.robot;

// Inside periodic:
import static edu.wpi.first.units.Units.*;

import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorLevelThreeCommand;
import frc.robot.commands.ElevatorLevelTwoAlgaeCommand;
import frc.robot.commands.ElevatorLevelTwoCommand;
import frc.robot.commands.FullAutoDeliverCoralTeleop;
import frc.robot.commands.GetCoralBabyBirdFromStationCommand;
import frc.robot.commands.GetCoralFromGroundSequentialCommand;
import frc.robot.commands.GetCoralSubstationCommand;
import frc.robot.commands.ManualClimbCommand;
import frc.robot.commands.NewAlageDown;
import frc.robot.AprilTagPositions;
import frc.robot.commands.AUTOAlgaeFromReef_PoolsideDelight;
import frc.robot.commands.AlgaeFromGroundPivotCommand;
import frc.robot.commands.AlgaeFromReefPivotCommand;
import frc.robot.commands.AlgaeLollipopPivotCommand;
import frc.robot.commands.AlgaePivotResetCommand;
import frc.robot.commands.AutoDeliverCoralTeleop;
import frc.robot.commands.BabyBirdFromStationSequentialCommand;
import frc.robot.commands.BensalemAlgaeClutch;
import frc.robot.commands.BlinkAlignCommand;
import frc.robot.commands.ClimbCurrentLimiter;
import frc.robot.commands.ClimberReleaseCommand;
import frc.robot.commands.ClimberSetCommand;
import frc.robot.commands.CoralFromGroundPivotCommand;
import frc.robot.commands.CoralLevelOnePivotCommand;
import frc.robot.commands.DeliverAlgaeBargeCommand;
import frc.robot.commands.DeliverCoralAtHeight;
import frc.robot.commands.DeliverCoralLevelOneSequentialCommand;
import frc.robot.commands.ElevatorBabyBirdCommand;
import frc.robot.commands.ElevatorCoralStationCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorLevelFourCommand;
import frc.robot.commands.ElevatorLevelOneCommand;
import frc.robot.commands.ElevatorLevelThreeAlgaeCommand;
//import frc.robot.commands.TrackToTagCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeLiberatorSubSystem;
import frc.robot.subsystems.AlgaePivotSubSystem;
import frc.robot.subsystems.CarriageSubSystem;
import frc.robot.subsystems.ClimberSubSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralAlignmentSubSystem;
//import frc.robot.subsystems.CoralHopperSubSystem;
//No longer using the HOPPER sub system
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;
import frc.robot.subsystems.LEDSubSystem;
import frc.robot.subsystems.WhaleTailReleaseSubSystem;
import frc.robot.subsystems.ChuteReleaseSubSystem;
import frc.robot.subsystems.LimelightSubsystem;

import static frc.robot.Constants.ReefPoses.*;
import static edu.wpi.first.wpilibj2.command.Commands.defer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class RobotContainer {
    



    // Swerve Set Up
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    // private final SwerveRequest.SwerveDriveBrake xWheels = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRR = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    private final CommandPS5Controller driver = new CommandPS5Controller(0);
    private final CommandPS5Controller operator = new CommandPS5Controller(1);

    //NEW TEST CONTROLLER BELOW:
    private final CommandPS5Controller tester = new CommandPS5Controller(2);

    // Subsystem Instantiation
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final CarriageSubSystem coralCarriage = new CarriageSubSystem();
    private final CoralOutakeSubSystem coralOutake = new CoralOutakeSubSystem();
    private final ElevatorSubSystem elevator = new ElevatorSubSystem();

    //private final CoralHopperSubSystem hopper = new CoralHopperSubSystem();
    //We are not using the hopper subsystem
    private final AlgaePivotSubSystem pivot = new AlgaePivotSubSystem();
    private final AlgaeLiberatorSubSystem liberator = new AlgaeLiberatorSubSystem();
    private final CoralAlignmentSubSystem align = new CoralAlignmentSubSystem();
    private final WhaleTailReleaseSubSystem whaleServo = new WhaleTailReleaseSubSystem();
    private final ChuteReleaseSubSystem chuteServo = new ChuteReleaseSubSystem();
    private final LEDSubSystem blink = new LEDSubSystem();
    private final ClimberSubSystem climber = new ClimberSubSystem();

    //Maps 
    Map<Integer, Pose2d> blueMap = AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS;
    Map<Integer, Pose2d> redMap = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS;
    public RobotContainer() {
        
      
        
        CameraServer.startAutomaticCapture(0);
        //Removed camera because of issues
        
        // Event Marker Commands for Path Planner
        NamedCommands.registerCommand("Station Pick Up Command", new GetCoralSubstationCommand(elevator, coralOutake, coralCarriage));
        //new GetCoralCommand(hopper, coralCarriage, coralOutake, elevator));
        //Removed this because we don't use the hopper anymore
        NamedCommands.registerCommand("L1", new ElevatorLevelOneCommand(elevator));
        NamedCommands.registerCommand("L2", new ElevatorLevelTwoCommand(elevator));
        NamedCommands.registerCommand("L4", new ElevatorLevelFourCommand(elevator));
        NamedCommands.registerCommand("Elevator Down", new ElevatorDownCommand(elevator));
        NamedCommands.registerCommand("Score At L4", new DeliverCoralAtHeight(new ElevatorLevelFourCommand(elevator), elevator, coralOutake, 69.8)); //70.65
        NamedCommands.registerCommand("Collect Algae L2", new AUTOAlgaeFromReef_PoolsideDelight(new ElevatorLevelTwoAlgaeCommand(elevator), liberator, pivot, elevator));
        NamedCommands.registerCommand("Collect Algae L3", new AUTOAlgaeFromReef_PoolsideDelight(new ElevatorLevelThreeAlgaeCommand(elevator), liberator, pivot, elevator));
        NamedCommands.registerCommand("Algae Elevator Down", new NewAlageDown(pivot, elevator, liberator, coralOutake));
        NamedCommands.registerCommand("Algae Barge Deliver", new DeliverAlgaeBargeCommand(new ElevatorLevelFourCommand(elevator), elevator, liberator, 71)); //Shoot at height needs to be adjusted
        NamedCommands.registerCommand("L4 Check", new ConditionalCommand(
            new ElevatorLevelFourCommand(elevator),
            new GetCoralSubstationCommand(elevator, coralOutake, coralCarriage),
            //new GetCoralCommand(hopper, coralCarriage, coralOutake, elevator),
            //Removed because we aren't using the hopper to intake anymore
            () -> elevator.getElevatorHeight() > 6.0
        ));

        // Auto Mode Set Up
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        //drivetrain.getPigeon2().setYaw(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180);
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            drivetrain.getPigeon2().setYaw(
                alliance.get() == Alliance.Blue ? 0 : 180
            );
        } else {
        // Default to Blue or some safe value in sim
            drivetrain.getPigeon2().setYaw(0);
}

        configureBindings();
    }
 

    private void configureBindings() {
        // Default Commands
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)//Changed back to positive
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)//Changed back to positive
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

        blink.setDefaultCommand(new BlinkAlignCommand(blink, align));

        drivetrain.registerTelemetry(logger::telemeterize);


        // DRIVER CONTROLS

        //driver.cross().whileTrue(new AlignToReefPoseCommand(drivetrain));

        //driver.R1().whileTrue((new TrackToTagCommand(drivetrain, limelight, blueMap, redMap)));

        driver.R1().whileTrue(
            new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetPoseBasedOnLL()),
            new WaitCommand(0.25),
            defer(
                () -> AutoBuilder.pathfindToPose(
                    drivetrain.getNearestReefPoseLeft(),
                    new PathConstraints(2, 2, 
                        540, 
                        720)
                ),
                    Set.of()
                )
            )
        );

        driver.L1().whileTrue(
            new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetPoseBasedOnLL()),
            new WaitCommand(0.25),
            defer(
                () -> AutoBuilder.pathfindToPose(
                    drivetrain.getNearestReefPoseRight(),
                    new PathConstraints(2, 2, 
                        540, 
                        720)
                ),
                    Set.of()
                )
            )
        );

        

        driver.cross().onTrue(new FullAutoDeliverCoralTeleop(2, new ElevatorLevelTwoCommand(elevator), elevator, coralOutake, 32.5, align, blink));
        driver.square().onTrue(new FullAutoDeliverCoralTeleop(3, new ElevatorLevelThreeCommand(elevator), elevator, coralOutake, 47.5, align, blink));
        driver.triangle().onTrue(new FullAutoDeliverCoralTeleop(4, new ElevatorLevelFourCommand(elevator), elevator, coralOutake, 69.8, align, blink));
        //driver.triangle().onTrue(new InstantCommand(()->{this.dropServo.dropTail(true);}));
        //Slow Mode
        driver.R2().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed * 0.25) //switched back
                                                               .withVelocityY(-driver.getLeftX() * MaxSpeed * 0.25)  //switched back
                                                               .withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.3)));
        // Robot Centric (at Slow Mode Speed)
        //driver.L2().whileTrue(drivetrain.applyRequest(() -> robotCentricDrive.withVelocityX(-driver.getLeftY() * MaxSpeed * 0.15)
                                                                               //.withVelocityY(-driver.getLeftX() * MaxSpeed * 0.15)
                                                                               //.withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.3)));

        // Field Centric Heading Reset
        //driver.circle().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //driver.circle().onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.fromDegrees(0))));


        //New Reset Rotation Method, Resets both pidgeon and pose
        driver.circle().onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(0.0)));

        //driver.L2().whileTrue(new RunCommand(()-> {liberator.setSpeed(1); (coralOutake.setSpeed(1);}, liberator, coralOutake))
                    //.onFalse(new InstantCommand(()-> {liberator.setSpeed(0); coralOutake.setSpeed(0);}, liberator, coralOutake));

        //driver.povDown().onTrue(new AlgaeFromGroundPivotCommand(pivot));
        //operator.L3().onTrue(new AlgaeFromGroundPivotCommand(pivot));
        //operator.R3().onTrue(new AlgaePivotResetCommand(pivot));
        //operator.axisLessThan(2, -0.75).onTrue(new CoralFromGroundPivotCommand(pivot));
        //operator.axisGreaterThan(2, 0.75).onTrue(new CoralLevelOnePivotCommand(pivot));
        operator.L3().onTrue(new CoralLevelOnePivotCommand(pivot))
                    .onFalse(new AlgaePivotResetCommand(pivot));
                //.onFalse(new SequentialCommandGroup (new InstantCommand(()-> {liberator.setSpeed(0);}, liberator), new AlgaePivotResetCommand(pivot)));
        //operator.axisLessThan(1, -0.75).onTrue(new BabyBirdFromStationSequentialCommand(liberator, elevator));
        operator.axisLessThan(1, -0.75).onTrue(new ElevatorBabyBirdCommand(elevator))
                                            .onFalse(new ElevatorDownCommand(elevator));
        //operator.axisGreaterThan(1, 0.75).onTrue(new GetCoralFromGroundSequentialCommand(pivot, liberator));

        operator.axisGreaterThan(1, 0.75).onTrue(new CoralFromGroundPivotCommand(pivot))
                                                        .onFalse(new AlgaePivotResetCommand(pivot));

        //operator.axisLessThan(5, -0.75).onTrue(new AlgaePivotResetCommand(pivot));
        operator.R3().onTrue(new AlgaePivotResetCommand(pivot));
        operator.axisGreaterThan(5, 0.75).onTrue(new AlgaeFromGroundPivotCommand(pivot));
        operator.axisLessThan(2, -0.75).onTrue(new AlgaeLollipopPivotCommand(pivot));
        //Lollipop Algae Command is UNTESTED

        //Climber Manual Commands  
        //UNTESTED!!!!!!!
        operator.axisGreaterThan(0, 0.1).and(operator.touchpad()).onTrue(new ClimbCurrentLimiter(climber, () -> {return operator.getRawAxis(0);}))
                                                                .onFalse(new InstantCommand(() -> {climber.setSpeed(0);},climber));

        //operator.axisGreaterThan(0, 0.1).and(operator.touchpad()).onTrue(new ManualClimbCommand(climber, () -> {return operator.getRawAxis(0);}))
                  //                      .onFalse(new InstantCommand(() -> {climber.setSpeed(0);},climber));
        operator.axisLessThan(0, -0.1).and(operator.touchpad()).onTrue(new ManualClimbCommand(climber, () -> {return operator.getRawAxis(0);}))
                                        .onFalse(new InstantCommand(() -> {climber.setSpeed(0);},climber));

        //Climber MANUAL Servo Commands    
        //operator.axisGreaterThan(2, 0.5).onTrue(new InstantCommand(() -> {chuteServo.releaseChute();}));
        //operator.axisLessThan(2, -0.5).onTrue(new InstantCommand(() -> {chuteServo.setChute();}));

        //operator.options().onTrue(new InstantCommand(()-> {whaleServo.releaseTail();}));
        //operator.touchpad().onTrue(new InstantCommand(()-> {whaleServo.setTail();}));

        //Climber AUTO Servo Commands
        //UNTESTED COMMANDS:
        operator.touchpad().and(operator.options()).onTrue(new ClimberReleaseCommand(chuteServo, whaleServo));
        //driver.povUp().onTrue(new ClimberSetCommand(chuteServo, whaleServo));
        driver.povUp().onTrue(new InstantCommand(()-> {whaleServo.setTail();}));  //seems to activate chute UPDATE: FIXED by changing the pin slots in the code, because they were wrong.
        driver.options().onTrue(new InstantCommand(() -> {chuteServo.setChute();})); // seems to activate whale tail UPDATE: Above.
        //operator.options().onTrue(new ClimberSetCommand(chuteServo, whaleServo));
        //Test Change

        //driver.povUp().onTrue(new AlgaePivotResetCommand(pivot));

        driver.povLeft().onTrue(new CoralLevelOnePivotCommand(pivot));

        driver.povRight().onTrue(new CoralFromGroundPivotCommand(pivot));

        // OPERATOR CONTROLS
        // Coral Elevator Bindings
        operator.cross().onTrue(new ElevatorCoralStationCommand(elevator));
        operator.square().onTrue(new ElevatorLevelTwoCommand(elevator));
        //Changed from Level 1 Elevator command to Coral Station command on 4/10
        operator.circle().onTrue(new ElevatorLevelThreeCommand(elevator));
        operator.triangle().onTrue(new ElevatorLevelFourCommand(elevator));
        //operator.square().onTrue(new FullAutoDeliverCoralTeleop(2, new ElevatorLevelTwoCommand(elevator), elevator, coralOutake, 32.5, align, blink));
        //operator.circle().onTrue(new FullAutoDeliverCoralTeleop(3, new ElevatorLevelThreeCommand(elevator), elevator, coralOutake, 47.5, align, blink));
        //operator.triangle().onTrue(new FullAutoDeliverCoralTeleop(4, new ElevatorLevelFourCommand(elevator), elevator, coralOutake, 69.8, align, blink));
        
        // Elevator Down
        operator.povDown().onTrue(new ElevatorDownCommand(elevator));

        // Run End Effector
        operator.L2().onTrue(new InstantCommand(() -> {coralCarriage.setSpeed(-0.75); coralOutake.setSpeed(-0.75);}, coralCarriage,coralOutake))
                    .onFalse(new InstantCommand(() -> {coralCarriage.setSpeed(0); coralOutake.setSpeed(0);},coralCarriage,coralOutake));

        operator.L1().onTrue(new InstantCommand(() -> {coralCarriage.setSpeed(0.45); coralOutake.setSpeed(0.45);}, coralCarriage,coralOutake))
                        .onFalse(new InstantCommand(() -> {coralCarriage.setSpeed(0); coralOutake.setSpeed(0);},coralCarriage,coralOutake));

        // Reverse End Effector
        //operator.R2().onTrue(new InstantCommand(() -> {coralCarriage.setSpeed(-1); coralOutake.setSpeed(-1);}, coralCarriage,coralOutake))
                        //.onFalse(new InstantCommand(() -> {coralCarriage.setSpeed(0); coralOutake.setSpeed(0);},coralCarriage,coralOutake));

        // Hopper
        //operator.povRight().onTrue(new GetCoralCommand(hopper, coralCarriage, coralOutake, elevator));
        //Removed this because the hopper is no longer being used
        operator.povRight().onTrue(new GetCoralSubstationCommand(elevator, coralOutake, coralCarriage));

        // Algae Bindings
        // ALGAE Elevator Commands
        operator.povLeft().onTrue(new BensalemAlgaeClutch(new ElevatorLevelTwoAlgaeCommand(elevator), pivot, liberator, coralOutake, elevator))
                                                .onFalse(new NewAlageDown(pivot, elevator, liberator, coralOutake));

        operator.povUp().onTrue(new BensalemAlgaeClutch(new ElevatorLevelThreeAlgaeCommand(elevator), pivot, liberator, coralOutake, elevator))
                                            .onFalse(new NewAlageDown(pivot, elevator, liberator, coralOutake));


        //TEST CONTROLLER CONTROLS FOR AUTO ALGAE COMMAND
        //UNTESTED
        tester.povDown().onTrue(new AUTOAlgaeFromReef_PoolsideDelight(new ElevatorLevelTwoAlgaeCommand(elevator), liberator, pivot, elevator));
        tester.povLeft().onTrue(new AUTOAlgaeFromReef_PoolsideDelight(new ElevatorLevelThreeAlgaeCommand(elevator), liberator, pivot, elevator));
        tester.povUp().onTrue(new DeliverAlgaeBargeCommand(new ElevatorLevelFourCommand(elevator), elevator, liberator, 71));

        //operator.triangle().and(operator.R1()).onTrue(new ElevatorLevelFourCommand(elevator));

        //Algae Outtake Bindings
        operator.R1().whileTrue(new RunCommand(()-> {liberator.setSpeed(-0.4); coralOutake.setSpeed(0);}, liberator, coralOutake))
            .onFalse(new InstantCommand(()-> {liberator.setSpeed(0); coralOutake.setSpeed(0);}, liberator, coralOutake));
        operator.R2().whileTrue(new RunCommand(()-> {liberator.setSpeed(1); coralOutake.setSpeed(1);}, liberator, coralOutake))
            .onFalse(new InstantCommand(()-> {liberator.setSpeed(Constants.DEFAULT_ALGEA_INTAKE); coralOutake.setSpeed(0);}, liberator, coralOutake));
        
        //Algae Intake Bindings
        //operator.L2().and(operator.R1()).whileTrue(new RunCommand(()-> {liberator.setSpeed(1); coralOutake.setSpeed(1);}, liberator, coralOutake))
            //.onFalse(new InstantCommand(()-> {liberator.setSpeed(0); coralOutake.setSpeed(0);}, liberator, coralOutake));


        //Algae Pivot Position Bindings
        //operator.povLeft().onTrue(new AlgaeFromReefPivotCommand(pivot));
        //operator.povUp().onTrue(new AlgaePivotResetCommand(pivot));

        // Back Up Driving Features
        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        )); */
        
        // Sys ID Routines
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        driver.triangle().and(driver.R1()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.triangle().and(driver.L1()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.square().and(driver.R1()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.square().and(driver.L1()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
