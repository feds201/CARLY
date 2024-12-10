package frc.robot.constants;

public class RobotMap {

    public static class SafetyMap {
        public static final double kMaxSpeed = 1.0;
        public static final double kMaxRotation = 1.0;
        public static final double kMaxAcceleration = 1.0;
        public static final double kMaxAngularAcceleration = 1.0;
        public static final double kJoystickDeadband = 0.1;
        public static final double kMaxSpeedChange = 0.5;

        public class FODC {
            public static final int LineCount = 72;
            public static double AngleDiff = 0.0;
        }
    }

    // USB Ports for Controllers
    public static class UsbMap {
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;
    }

    // CAN IDs for Swerve Drive System
    public static class SwerveMap {
        public static final int FRONT_LEFT_STEER = 0;
        public static final int FRONT_RIGHT_STEER = 1;
        public static final int BACK_LEFT_STEER = 2;
        public static final int BACK_RIGHT_STEER = 3;
        public static final int FRONT_LEFT_DRIVE = 4;
        public static final int FRONT_RIGHT_DRIVE = 5;
        public static final int BACK_LEFT_DRIVE = 6;
        public static final int BACK_RIGHT_DRIVE = 7;
    }

    public static class ElevatorMap {
        public static final int ELEVATOR_MOTOR = 8;
    }

    public static class LauncherMap
    {
        public static final int LAUNCHER_MOTOR = 9;
        public static final int TURRET_MOTOR = 10;
        public static final int LAUNCHER_SOLENOID_FORWARD = 3;
        public static final int LAUNCHER_SOLENOID_REVERSE = 4;
    }

    // Additional motor controllers or sensors could be added here
    public static class SensorMap {
        // Example: Add sensor ports (like encoders, gyros, etc.)
        public static final int GYRO_PORT = 0;
    }


    public static class IntakeMap{
        //Make sure to change these to correct!
        public static final int kIntakeWheels = 50;       // Rev
        public static final int kIntakeWrist = 55;        // Rev
    }
    // You can add more mappings for other subsystems like intake, shooter, etc.
}
