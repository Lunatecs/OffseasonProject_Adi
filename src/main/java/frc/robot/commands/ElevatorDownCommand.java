// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDownCommand extends Command {
    private ElevatorSubSystem elevator;
    private PIDController controller;
    private boolean isFinished;
  /** Creates a new ElevatorDownCommand. */
  public ElevatorDownCommand(ElevatorSubSystem elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
    controller = new PIDController(0.05, 0, 0); //0.0139, 0.017375, 0.019, 0.022, 0.0275, 0.031
    controller.setSetpoint(elevator.getElevatorHeight());
    controller.setTolerance(0);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(elevator.getElevatorHeight());
    if(Math.abs(speed)> 0.5){
      speed = Math.signum(speed) * 0.5;
    }
    if (elevator.getElevatorHeight()<15.0){
      speed = -0.1;
    }
    if(elevator.getLimitSwitch()) {
      speed = 0.0;
      isFinished = true;
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
    return isFinished;
  }
}
