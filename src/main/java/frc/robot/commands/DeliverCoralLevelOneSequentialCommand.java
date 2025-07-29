// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeLiberatorSubSystem;
import frc.robot.subsystems.AlgaePivotSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeliverCoralLevelOneSequentialCommand extends SequentialCommandGroup {
  /** Creates a new DeliverCoralL1SequentialCommand. */
  public DeliverCoralLevelOneSequentialCommand(AlgaePivotSubSystem pivot, AlgaeLiberatorSubSystem roller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CoralLevelOnePivotCommandEndable(pivot),
      new InstantCommand(()-> {roller.setSpeed(0.75);}, roller)
    );
  }
}
