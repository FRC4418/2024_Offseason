// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.server.PathPlannerServer;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  
  public DifferentialDrivePoseEstimator estimator;
//  The motors on the left side of the drive.
  public final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(4);
  public final WPI_TalonFX leftBackMotor = new WPI_TalonFX(1);
  public MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);

  public final WPI_TalonFX rightFrontMotor = new WPI_TalonFX(3);
  public final WPI_TalonFX rightBackMotor = new WPI_TalonFX(2);

  private int inverted = 0;

  private autoBalance balancer = new autoBalance();

  private double speed;
  
  // final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(38);
  // final WPI_TalonFX leftBackMotor = new WPI_TalonFX(00);
  // MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);

  // final WPI_TalonFX rightFrontMotor = new WPI_TalonFX(36);
  // final WPI_TalonFX rightBackMotor = new WPI_TalonFX(39);
  public MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  public DifferentialDriveKinematics kinematics;

  // The left-side drive encoder

  // The gyro sensor
  //private final Gyro m_gyro = new ADXRS450_Gyro();
  public final AHRS ahrs = new AHRS();

  private PIDController leftPID = new PIDController(
    Constants.AutoPIDs.kP,
    Constants.AutoPIDs.kI,
    Constants.AutoPIDs.kD);
private PIDController rightPID = new PIDController(
    Constants.AutoPIDs.kP,
    Constants.AutoPIDs.kI,
    Constants.AutoPIDs.kD);


  
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    ahrs.reset();
    ahrs.calibrate();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(false);
    m_leftMotors.setInverted(true);

    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftBackMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightBackMotor.setNeutralMode(NeutralMode.Brake);

    // Sets the distance per pulse for the encoders

    kinematics = new DifferentialDriveKinematics(0.6096);


    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
  
    estimator = new DifferentialDrivePoseEstimator(kinematics, ahrs.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance(), new Pose2d());

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    //getWheelSpeeds();
    //System.out.println(getLeftEncoderDistance());
    //System.out.println(getRightEncoderDistance());
    estimator.update(ahrs.getRotation2d()
    , getLeftEncoderDistance(), getRightEncoderDistance());
    SmartDashboard.putString("estimator pose", estimator.getEstimatedPosition().toString());
    SmartDashboard.putNumber("Speed from balancer", speed);
    System.out.println(ahrs.getRotation2d().toString());
  }

  public void changeInvert(){
    m_rightMotors.setInverted(true);
    m_leftMotors.setInverted(false);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  private double nativeUnitsToDistanceMeters(double sensorCoubts){
    double motorRotations = (double)sensorCoubts / DriveConstants.kEncoderCPR;
    double wheelRoations = motorRotations / 6;
    double positionMeters = wheelRoations * (2 * Math.PI * Units.inchesToMeters(2));
    return positionMeters;
  }

  
  public void tankDrive(double left, double right) {
    
    m_drive.tankDrive(left, right, false);
  }

public void curvatureDrive(double xSpeed, double zRotation, double baseTS) {
    // Clamp all inputs to valid valuess
    xSpeed = SLMath.clamp(xSpeed, -1.0, 1.0);
    zRotation = SLMath.clamp(zRotation, -1.0, 1.0);
    baseTS = SLMath.clamp(baseTS, 0.0, 1.0);

    // Find the amount to slow down turning by.
    // This is proportional to the speed but has a base value
    // that it starts from (allows turning in place)
    double turnAdj = Math.max(baseTS, Math.abs(xSpeed));

    // Find the speeds of the left and right wheels
    double lSpeed = xSpeed + zRotation * turnAdj;
    double rSpeed = xSpeed - zRotation * turnAdj;

    // Find the maximum output of the wheels, so that if a wheel tries to go > 1.0
    // it will be scaled down proportionally with the other wheels.
    double scale = Math.max(1.0, Math.max(Math.abs(lSpeed), Math.abs(rSpeed)));

    lSpeed /= scale;
    rSpeed /= scale;

    // Feed the inputs to the drivetrain
    tankDrive(lSpeed, rSpeed);
}

public void curvatureDrive(double xSpeed, double zRotation) {
  this.curvatureDrive(xSpeed, zRotation, 0.45);
}

public void impulseDrive(double xSpeed, double zRotation) {
    // If the speed is negative and the steering setpoint is small, then invert the
    // steering controls
    if (xSpeed < -0.05 && Math.abs(zRotation) < 0.15) {
      curvatureDrive(xSpeed, zRotation); // Inverted steering
    } else {
      curvatureDrive(xSpeed, -zRotation); // Standard steering
    }
  }


  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    double leftWheelSpeeds = -10 * nativeUnitsToDistanceMeters(leftBackMotor.getSelectedSensorVelocity());
    double rightWheelSpeeds = 10 * nativeUnitsToDistanceMeters(rightBackMotor.getSelectedSensorVelocity());
    System.out.println("left:" + leftWheelSpeeds + "       right:" + rightWheelSpeeds);
    return new DifferentialDriveWheelSpeeds(leftWheelSpeeds, rightWheelSpeeds);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void  resetOdometry(Pose2d pose) {
    //m_odometry.resetPosition(ahrs.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance(), new Pose2d());
    
    estimator.resetPosition(ahrs.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance(), pose);
    resetEncoders();
    /**
    PROTOTYPE, set the wheel position to the x value of the pose and rotation to the rot. value of
    supplied pose
    **/
    //estimator.resetPosition(pose.getRotation(), pose.getX(), pose.getX(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    rightBackMotor.setSelectedSensorPosition(0);
    leftBackMotor.setSelectedSensorPosition(0);
    ahrs.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoderDistance() {
    return -nativeUnitsToDistanceMeters(leftBackMotor.getSelectedSensorPosition());
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoderDistance() {
    return nativeUnitsToDistanceMeters(rightBackMotor.getSelectedSensorPosition());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return ahrs.getRotation2d().getDegrees()+360;
  }

  public void autoBalanceDrive(){
    arcadeDrive(speed,0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return ahrs.getRate();
  }

  public Command drivePath(boolean isFirstPath, String nameOfPath) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(nameOfPath);
    PathPlannerLogging.logActivePath(path);

    return new FollowPathWithEvents(
      new FollowPathRamsete(
        path, 
        this::getPose,  
        this::getWheelSpeeds 
        this::tankDriveVolts, 
        new ReplanningConfig(), 
        this
      ), 
      path,
      this::getPose
    );    
  }
  

}
