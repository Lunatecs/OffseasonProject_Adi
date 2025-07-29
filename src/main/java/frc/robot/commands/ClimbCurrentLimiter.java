// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCurrentLimiter extends Command {
  /** Creates a new ClimbCurrentLimiter. */
  ClimberSubSystem climber;
  boolean isFinished;
  DoubleSupplier power;
  public ClimbCurrentLimiter(ClimberSubSystem climber, DoubleSupplier power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.power = power;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.getCurrent() >= Constants.ClimberSubSystemConstants.climbCurrentLimit) {  
      isFinished = true;
    } else {
      climber.setSpeed(power.getAsDouble());
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
