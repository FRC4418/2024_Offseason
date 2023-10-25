// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  /** Creates a new rollers. */
  final WPI_TalonFX rollerSpinner = new WPI_TalonFX(21);
  public Rollers() {
    rollerSpinner.configFactoryDefault();
    rollerSpinner.setSelectedSensorPosition(0);
    rollerSpinner.setNeutralMode(NeutralMode.Brake);
  }

  public void intakeSpin(double speed){
    rollerSpinner.set(ControlMode.PercentOutput, speed);
}

public void intakeStop(){
    rollerSpinner.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
