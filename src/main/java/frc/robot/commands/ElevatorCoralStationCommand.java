// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCoralStationCommand extends Command {

  private ElevatorSubSystem elevator;
  private PIDController controller;

  /** Creates a new ElevatorSixFeetCommand. */
  public ElevatorCoralStationCommand(ElevatorSubSystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
    controller = new PIDController(0.03, 0, 0); //0.01025
    controller.setSetpoint(20.0); //19 as of 4:55 PM 4/17/25
    controller.setTolerance(0.25);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(elevator.getElevatorHeight());
    if(Math.abs(speed)> 1.0){
      speed = Math.signum(speed) * 1.0;
    }
    elevator.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
