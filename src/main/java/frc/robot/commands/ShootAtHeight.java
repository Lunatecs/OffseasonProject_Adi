// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Date;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAtHeight extends Command {
  private CoralOutakeSubSystem outtake;
  private double height;
  private boolean isFinished;
  private Date start;
  private ElevatorSubSystem elevator;
  /** Creates a new ShootAtHeight. */
  public ShootAtHeight(CoralOutakeSubSystem outtake, ElevatorSubSystem elevator, double height) {
    this.outtake = outtake;
    this.elevator = elevator;
    this.height = height;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outtake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.getElevatorHeight()>= height){
      
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
