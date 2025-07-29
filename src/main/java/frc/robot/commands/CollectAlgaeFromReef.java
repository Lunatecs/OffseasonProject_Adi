// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeLiberatorSubSystem;
import frc.robot.subsystems.AlgaePivotSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectAlgaeFromReef extends ParallelDeadlineGroup {
  /** Creates a new CollectAlgaeFromReef. */
  public CollectAlgaeFromReef(AbstractAlgaeElevatorCommand elevatorCommand, AlgaePivotSubSystem pivot, AlgaeLiberatorSubSystem roller, CoralOutakeSubSystem coralOutake, ElevatorSubSystem elevator) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    //super(new HasAlgae(roller));
    // addCommands(new FooCommand(), new BarCommand());
    super(
      elevatorCommand,
      new SequentialCommandGroup(
      new WaitTillSetpointElevatorCommand(elevator, elevatorCommand.getSetpoint()),
      new AlgaeFromReefPivotCommand(pivot)),
      new RunCommand(()-> {roller.setSpeed(0.8); coralOutake.setSpeed(0.4);}, roller, coralOutake));
      //NEED TO TEST REMOVING CORAL OUTTAKE, Stopped running it in the new RunAlgaeTillCurrent command
      /*new ParallelDeadlineGroup(
        new HasAlgae(roller),
        new RunCommand(()-> {roller.setSpeed(1.0); coralOutake.setSpeed(0.4);}, roller, coralOutake)));*/
      
    addRequirements(elevator, pivot, roller, coralOutake);
  }
}
