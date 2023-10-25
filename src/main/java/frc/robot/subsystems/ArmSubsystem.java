package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ArmSubsystem extends SubsystemBase {
    boolean isAtHome = false;

    final WPI_TalonFX intakeMaster = new WPI_TalonFX(20);
    final WPI_TalonFX intakeSlave = new WPI_TalonFX(22); 

    private int peakVelocityUp = 4600;
    private final double percentOfPeakUp = .75;
    private final double upkF = (percentOfPeakUp * 2048) / (peakVelocityUp * percentOfPeakUp);
    private final double cruiseVelocityAccelUp = peakVelocityUp * percentOfPeakUp;

    private int peakVelocityDown = 33090;
    private final double percentOfPeakDown = .35;
    private final double downkF = (percentOfPeakDown * 2048) / (peakVelocityDown * percentOfPeakDown);
    private final double cruiseVelocityAccelDown = peakVelocityDown * percentOfPeakDown;

    public ArmSubsystem() {
        intakeMaster.configFactoryDefault();
        intakeMaster.setSelectedSensorPosition(0);

		intakeMaster.config_kF(0, 0.1, 0);
		intakeMaster.config_kP(0, 0.06030624264, 0);
		intakeMaster.config_kI(0, 0, 0);
		intakeMaster.config_kD(0, 0, 0);

        intakeMaster.config_kF(1, 0.1, 0);
		intakeMaster.config_kP(1, 0.1265760198, 0);
		intakeMaster.config_kI(1, 0, 0);
		intakeMaster.config_kD(1, 0, 0);

        intakeMaster.configMotionSCurveStrength(2);

        intakeMaster.setInverted(true);
        intakeMaster.setNeutralMode(NeutralMode.Brake);
        intakeSlave.setNeutralMode(NeutralMode.Brake);

        //intakeSlave.configRemoteFeedbackFilter(9, RemoteSensorSource.TalonFX_SelectedSensor, 0);
        //intakeSlave.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Master Falcon Position", intakeMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Slave Falcon Position", intakeSlave.getSelectedSensorPosition());
    }

    public void motorPositionControl(int position) {
        
    }



    public void goToHome() {
        intakeMaster.set(TalonFXControlMode.Position, 100);
        intakeSlave.set(TalonFXControlMode.Position, 100);
    }

    public void dumbGoToHome(){
            if(intakeMaster.getSelectedSensorPosition() < -8000){
                intakeMaster.set(0.4);
                intakeSlave.follow(intakeMaster);
                isAtHome = false;
            } else {
                isAtHome = true;
            }
    }

    public void slowlyGoDown() {
        intakeMaster.set( -0.4);
        intakeSlave.follow(intakeMaster);
        intakeSlave.setInverted(InvertType.OpposeMaster);
    }

    public Command slowyGoUp() {
        return runOnce(
            () -> {
                intakeMaster.set(TalonFXControlMode.PercentOutput, .1);
                intakeSlave.follow(intakeMaster);
                intakeSlave.setInverted(InvertType.OpposeMaster);
            }
        );
    }

    public void stop(){
                intakeMaster.set(TalonFXControlMode.PercentOutput, 0);
                intakeSlave.set(TalonFXControlMode.PercentOutput, 0);
    }

    public Command resetSensor() {
        return runOnce(
            () -> {
                intakeMaster.setSelectedSensorPosition(0);
                intakeSlave.setSelectedSensorPosition(0);
            }
        );
    }

    public Command pickUpOnGround() {
        return runOnce(
            () -> {
                intakeMaster.setSelectedSensorPosition(0);
                intakeSlave.setSelectedSensorPosition(0);
            }
        );
    }

    public void setPosition(double position) {
        manageMotion(position);
        intakeMaster.set(ControlMode.MotionMagic, position);
        intakeSlave.follow(intakeMaster);
        intakeSlave.setInverted(InvertType.OpposeMaster);
    }

    public Command setVoltage(float voltage) {
        return runOnce(
            () -> {
                intakeMaster.set(ControlMode.PercentOutput, voltage);
                intakeSlave.follow(intakeMaster);
                intakeSlave.setInverted(InvertType.OpposeMaster);
            }
        );
    }
    
    public void manageMotion(double targetPosition) {
        double currentPosition = intakeMaster.getSelectedSensorPosition();
    
        // going up
        if(currentPosition < targetPosition) {
    
          // set accel and velocity for going up
          intakeMaster.configMotionAcceleration(cruiseVelocityAccelUp, 0);
          intakeMaster.configMotionCruiseVelocity(cruiseVelocityAccelUp, 0);
    
          // select the up gains
          intakeMaster.selectProfileSlot(0, 0);
          SmartDashboard.putBoolean("Going Up or Down", true);
    
        } else {
          
          // set accel and velocity for going down
          intakeMaster.configMotionAcceleration(cruiseVelocityAccelDown, 0);
          intakeMaster.configMotionCruiseVelocity(cruiseVelocityAccelDown, 0);
    
          // select the down gains
          intakeMaster.selectProfileSlot(1, 0);
          SmartDashboard.putBoolean("Going Up or Down", false);

        }
    
      }

      public boolean isAtHome(){
        return isAtHome;
      }

      public double getMasterPos(){
        return intakeMaster.getSelectedSensorPosition();
      }
}