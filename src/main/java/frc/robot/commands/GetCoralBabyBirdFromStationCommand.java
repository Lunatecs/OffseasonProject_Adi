// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.AlgaeLiberatorSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetCoralBabyBirdFromStationCommand extends ParallelDeadlineGroup {
  /** Creates a new GetCoralBabyBirdFromStationCommand. */
  public GetCoralBabyBirdFromStationCommand(AlgaeLiberatorSubSystem roller, ElevatorSubSystem elevator) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new HasCoralLevelOne(roller),
    new ElevatorBabyBirdCommand(elevator),
    new RunCommand(()-> {roller.setSpeed(-0.4);}, roller)
    );
    // addCommands(new FooCommand(), new BarCommand());
  }
}
