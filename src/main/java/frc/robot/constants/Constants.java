// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do n  ot put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 0;
    public static final int kLeftMotor2Port = 1;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;

    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    //ACUTALY SYSID VALUES
    // public static final double ksVolts = 0.6871;
    // public static final double kvVoltSecondsPerMeter = 1.98;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.61067;
    //TEST GAINS
     public static final double ksVolts = 0.3471;
     public static final double kvVoltSecondsPerMeter = 0.98;
     public static final double kaVoltSecondsSquaredPerMeter = 0.31067;



    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
  }

  public static final class visionTrajEndPoint{
    public static final double xOffset = 0.8;
    public static final double yOffset = 0.0;
  }

  public static final class intakePositionControl{
    public static final int downPos = -5700;
    public static final int farBackPos = -2200;
    public static final int conePos = -4000;
    public static final double pullSpeed = 0.05;
    public static final double pushSpeed = -0.05;
  } 

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class driveSpeeds{
    public static final double maxSpeed = 0.5;
  }

  public static final class AutoPIDs{
    public static final double kP = 5.155;
    public static final double kI = 0.1;
    public static final double kD = 0.05;
  }

public static Transform3d kCameraToRobot = new Transform3d(new Translation3d(0.56,0.25,0), new Rotation3d(0,0,0));
}
