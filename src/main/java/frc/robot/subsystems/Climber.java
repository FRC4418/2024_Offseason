// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new rollers. */
  final WPI_TalonFX climber = new WPI_TalonFX(31);
  public Climber() {
    climber.configFactoryDefault();
    climber.setSelectedSensorPosition(0);
    climber.setNeutralMode(NeutralMode.Brake);
  }

  public void climberSpin(double speed){
    climber.set(speed);
}

public void climberStop(){
    climber.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
