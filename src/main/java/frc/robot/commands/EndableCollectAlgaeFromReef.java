// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Date;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeLiberatorSubSystem;
import frc.robot.subsystems.AlgaePivotSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EndableCollectAlgaeFromReef extends ParallelDeadlineGroup {
  /** Creates a new EndableCollectAlgaeFromReef. */
  Date start = new Date();
  public EndableCollectAlgaeFromReef(AbstractAlgaeElevatorCommand elevatorCommand, AlgaePivotSubSystem pivot, AlgaeLiberatorSubSystem roller, CoralOutakeSubSystem coralOutake, ElevatorSubSystem elevator) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(
      new WaitCommand(1.0) //not correct just put to satisfy requirements of super
      );
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    elevatorCommand,
    new SequentialCommandGroup(
    new WaitTillSetpointElevatorCommand(elevator, elevatorCommand.getSetpoint()),
    new AlgaeFromReefPivotCommand(pivot)),
    new RunCommand(()-> {roller.setSpeed(0.8); coralOutake.setSpeed(0.4);}, roller, coralOutake));
  }
}
