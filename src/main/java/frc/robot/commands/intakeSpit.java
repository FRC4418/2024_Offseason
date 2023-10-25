// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.stuypulse.stuylib.util.plot.Playground.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rollers;

public class intakeSpit extends Command {
  /** Creates a new intakeSuck. */
  Rollers rollers = new Rollers();
  double speed;

  public intakeSpit(Rollers rollers, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollers);
    this.rollers = rollers;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rollers.intakeSpin(speed);
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
