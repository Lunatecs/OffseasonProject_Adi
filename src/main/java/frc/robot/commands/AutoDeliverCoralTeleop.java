// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralAlignmentSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;
import frc.robot.subsystems.LEDSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDeliverCoralTeleop extends ParallelDeadlineGroup {
  /** Creates a new DeliverCoralAtHeight. */
  public AutoDeliverCoralTeleop(int level, Command goToLevelCommand, ElevatorSubSystem elevator, CoralOutakeSubSystem coralOutake, double shootAtHeight, CoralAlignmentSubSystem align, LEDSubSystem led) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new SequentialCommandGroup(
            new WaitUntilAtHeightCommand(elevator, shootAtHeight),
            //new SensorAlignToPole(level, align, led),
            new ShootCoralCommand(coralOutake)));
    addCommands(goToLevelCommand);
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(elevator,coralOutake);
  }
}
