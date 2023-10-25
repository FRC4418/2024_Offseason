// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class balance extends CommandBase {
  /** Creates a new balance. */
  private DriveSubsystem driveTrain;
  private double currentPitch;
  private double pitchThreshhold = 0.6;
  private double velocity;
  private boolean isFinished;
  private int z = 0;
  public balance(DriveSubsystem driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPitch = driveTrain.ahrs.getPitch();

    if(Math.abs(currentPitch) > pitchThreshhold){
      velocity = (-(currentPitch) / 65);
      driveTrain.tankDrive(-velocity, -velocity);
    } else if(Math.abs(currentPitch) > pitchThreshhold && Math.abs(currentPitch) < 3d){
      velocity = (-(currentPitch) / 20);
    } else if(currentPitch > pitchThreshhold && currentPitch < 0.3){
      driveTrain.tankDrive(velocity,velocity);
    } else {
    if (z == 100) {
      driveTrain.tankDrive(0,0);
      z = 0;
  }
    else {
      z += 2;
  }
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
