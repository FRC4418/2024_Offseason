// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Settings;
import frc.robot.subsystems.DriveSubsystem;

public class DrivetrainDrive extends Command {
  private final DriveSubsystem drivetrain;
  private final Gamepad driver;

  private final IStream speedSetpoint, angleSetpoint;

  public DrivetrainDrive(DriveSubsystem drivetrain, AutoGamepad driver2) {
    this.drivetrain = drivetrain;
    this.driver = driver2;

    // Gives 1 to -1, and 0 when both triggers are held down
    // Mapped to symetric max values from shuffleboard
    this.speedSetpoint = IStream.create(() -> driver2.getRightTrigger() - driver2.getLeftTrigger())
        .filtered(
            x -> SLMath.map(x, -1, 1, -Settings.Drivetrain.MAX_SPEED.get(), Settings.Drivetrain.MAX_SPEED.get()),
            x -> SLMath.deadband(x, Settings.Drivetrain.SPEED_DEADBAND.get()),
            x -> SLMath.spow(x, Settings.Drivetrain.SPEED_POWER.get()),
            new LowPassFilter(Settings.Drivetrain.SPEED_FILTER));

    this.angleSetpoint = IStream.create(() -> -driver2.getLeftX())
        .filtered(
            x -> SLMath.map(x, -1, 1, Settings.Drivetrain.MAX_SPEED_ANGLE.get(),
                -Settings.Drivetrain.MAX_SPEED_ANGLE.get()),
            x -> SLMath.deadband(x, Settings.Drivetrain.ANGLE_DEADBAND.get()),
            x -> SLMath.spow(x, Settings.Drivetrain.ANGLE_POWER.get()),
            new LowPassFilter(Settings.Drivetrain.ANGLE_FILTER));

    addRequirements(drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.impulseDrive(speedSetpoint.get(), angleSetpoint.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
