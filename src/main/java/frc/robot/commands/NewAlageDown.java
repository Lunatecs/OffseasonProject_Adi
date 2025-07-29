// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeLiberatorSubSystem;
import frc.robot.subsystems.AlgaePivotSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewAlageDown extends ParallelCommandGroup {
  /** Creates a new NewAlageDown. */
  public NewAlageDown(AlgaePivotSubSystem pivot, ElevatorSubSystem elevator, AlgaeLiberatorSubSystem roller, CoralOutakeSubSystem outake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlgaePivotResetCommand(pivot),
      new InstantCommand(()-> {roller.setSpeed(Constants.DEFAULT_ALGEA_INTAKE); outake.setSpeed(0);}, roller, outake),
      new ElevatorDownCommand(elevator)
    );
  }
}
