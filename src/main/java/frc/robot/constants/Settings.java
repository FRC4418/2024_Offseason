package frc.robot.constants;


// import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.network.SmartString;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;


/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Drivetrain {
        // If speed is below this, use quick turn
        SmartNumber BASE_TURNING_SPEED = new SmartNumber("Driver Settings/Base Turn Speed", 0.45);
        SmartNumber INVERT_ANGLE_THREASHOLD = new SmartNumber(
        "Driver Settings/ Invert Steering, Angle Setpoint Threshold", 0.15);

        // Low Pass Filter and deadband for Driver Controls
        SmartNumber SPEED_DEADBAND = new SmartNumber("Driver Settings/Speed Deadband", 0.00);
        SmartNumber ANGLE_DEADBAND = new SmartNumber("Driver Settings/Turn Deadband", 0.10);
        
        SmartNumber MAX_SPEED_ANGLE = new SmartNumber("Driver Settings/Max Speed Angle", 0.85);
        SmartNumber MAX_SPEED = new SmartNumber("Driver Settings/Max Speed", 1.0);

        SmartNumber SPEED_POWER = new SmartNumber("Driver Settings/Speed Power", 2.0);
        SmartNumber ANGLE_POWER = new SmartNumber("Driver Settings/Turn Power", 1.0);

        SmartNumber SPEED_FILTER = new SmartNumber("Driver Settings/Speed Filtering", 0.25);
        SmartNumber ANGLE_FILTER = new SmartNumber("Driver Settings/Turn Filtering", 0.005);

        SmartNumber DISPLACEMENT_METERS = new SmartNumber("Driver Settings/Displacement Auto", 0.0);
        SmartNumber TIME_MOVING = new SmartNumber("Driver Settings/Time Driving Auto", 0.0);

        // Width of the robot
        double TRACK_WIDTH = Units.inchesToMeters(26.9); 

        public interface Motion {

            DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

            public interface FeedForward {
                double kS = 0.20094;
                double kV = 1.6658;
                double kA = 0.4515;
            }

            public interface PID {
                int kSlot = 0;
                double kF = 0;
                double kP = 0;
                double kI = 0;
                double kD = 0;
                double kTimeoutMs = 50;
            }
        }

        // Encoder Constants
        public interface Encoders {

            public interface GearRatio {

                public interface Stages {
                    double FIRST_STAGE = (8.0 / 60.0);

                    double SECOND_STAGE = (1.0 / 1.0);
                }

                double ENCODER_TO_WHEEL = Stages.FIRST_STAGE * Stages.SECOND_STAGE;
            }

            double WHEEL_DIAMETER = Units.inchesToMeters(4);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

            double ENCODER_PULSES_PER_REVOLUTION = 2048;
            double ENCODER_DISTANCE_PER_PULSE =
                    (WHEEL_CIRCUMFERENCE / ENCODER_PULSES_PER_REVOLUTION)
                            * GearRatio.ENCODER_TO_WHEEL;
        }
    }

    public interface Intake{
        SmartNumber SPIN_SPEED = new SmartNumber("Feeder and Shooter Settings/Roller Spin Speed", 0.75);

        SmartNumber INTAKE_POSITION_UP = new SmartNumber("Feeder and Shooter Settings/Intake Position Up", 2000.0);

        SmartNumber INTAKE_POSITION_DOWN = new SmartNumber("Feeder and Shooter Settings/Intake Position Down", 16500.0);

        SmartNumber UPPER_SHOOT_SPEED = new SmartNumber("Feeder and Shooter Settings/Upper Intake Shoot Speed", 0.50);

        public interface PID {
            int kSlot = 0;
            double kF = 0;
            double kP = 0.02;
            double kI = 0.00001;
            double kD = 0.001;
            double kTimeoutMs = 50;
        }
    }

    public interface Shooter {
        // Low Pass Filter and deadband for Feeder Controls
        SmartNumber SHOOT_SPEED = new SmartNumber("Feeder and Shooter Settings/Shooting Speed", 23.0);

        public interface PID {
            int kSlot = 0;
            double kF = 0;
            double kP = 0.005;
            double kI = 0;
            double kD = 0;
            double kTimeoutMs = 50;
        }
    }
    public interface Climber {
        SmartNumber WINCH_POWER = new SmartNumber("Climber Settings/Climber Speed", 0.75);

        SmartNumber RATCHET_ENGAGE_ANGLE = new SmartNumber("Climber Settings/Ratchet Engage Angle", 40);

        SmartNumber RATCHET_RELEASE_ANGLE = new SmartNumber("Climber Settings/Ratchet Release Angle", 0);
    }

}
