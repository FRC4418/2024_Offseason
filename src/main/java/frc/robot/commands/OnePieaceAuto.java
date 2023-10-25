// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.constants.Settings;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Rollers;
//1 PIEACE AUTO SHORT
public class OnePieaceAuto extends CommandBase {
  /** Creates a new OnePieaceAuto. */
  private DriveSubsystem driveTrain;
  private Rollers rollers;
  private ArmSubsystem arm;
  public OnePieaceAuto(DriveSubsystem driveTrain, Rollers rollers, ArmSubsystem arm) {
    this.driveTrain = driveTrain;
    this.rollers = rollers;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, rollers, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new SequentialCommandGroup(new intakeSpinAuto(rollers, 0.5), driveTrain.drivePath(isFinished(), "1 Piece Short"), new moveIntakePos(arm, Constants.intakePositionControl.downPos));
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
