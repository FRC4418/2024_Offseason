package frc.robot.constants;

public interface Ports {

    public interface Gamepad{
        int DRIVER = 0;
        int OPERATOR = 1;
    }

    public interface Intake{
        int INTAKE_LEFT = 2;
        int INTAKE_RIGHT = 3;
    }

    public interface Arms{
        int ARMS = 4;
    }

    public interface Drivetrain {
        // Motors
        int LEFT_FRONT = 2;
        int LEFT_BACK = 3;

        int RIGHT_FRONT = 5;
        int RIGHT_BACK = 6;
    }
    
    
}
