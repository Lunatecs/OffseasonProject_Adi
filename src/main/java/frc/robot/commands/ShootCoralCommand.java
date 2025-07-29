// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Date;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralOutakeSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCoralCommand extends Command {
  private CoralOutakeSubSystem outtake;
  private Date start;
  private Date secondary;
  boolean isFinished;

  /** Creates a new ShootCoralCommand. */
  public ShootCoralCommand(CoralOutakeSubSystem outtake) {
    this.outtake = outtake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outtake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = new Date();
    secondary = null;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    long now = new Date().getTime();
    long runningTime = now - start.getTime();

    if (outtake.hasCoral() || runningTime < 300L) {
      outtake.setSpeed(0.4);
    } else if (!outtake.hasCoral() && secondary == null) {
      secondary = new Date();
      outtake.setSpeed(1.0);
    } else if (secondary != null && (now - secondary.getTime() < 400L)) {
      outtake.setSpeed(1.0);
    } else {
      isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    outtake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
