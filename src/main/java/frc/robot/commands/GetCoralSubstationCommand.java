// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CarriageSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetCoralSubstationCommand extends SequentialCommandGroup {
  /** Creates a new GetCoralSubstationCommand. */
  public GetCoralSubstationCommand(ElevatorSubSystem elevator, CoralOutakeSubSystem outake, CarriageSubSystem carriage) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup(new ReceiveCoralCommand(outake, carriage), new ElevatorCoralStationCommand(elevator)),
        new ParallelDeadlineGroup(
          new WaitCommand(0.25), //0.25
          new InstantCommand(()->{outake.setSpeed(-1);},outake)),
        new InstantCommand(()->{outake.setSpeed(0);},outake),
        new ElevatorCoralStationCommand(elevator)
    );
    addRequirements(elevator, outake, carriage);
  }
}
