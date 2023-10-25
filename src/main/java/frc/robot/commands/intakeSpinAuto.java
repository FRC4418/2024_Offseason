// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Rollers;

public class intakeSpinAuto extends CommandBase {
  /** Creates a new intakeSpitAuto. */
  private Rollers rollers;
  private Timer timer;
  private double speed;
  public intakeSpinAuto(Rollers rollers, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollers);
    this.rollers = rollers;
    timer = new Timer();
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rollers.intakeSpin(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollers.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > 0.4){
      return true;
    } else {
      return false;
    }
  }
}
