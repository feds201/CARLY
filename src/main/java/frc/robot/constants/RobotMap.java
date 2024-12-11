package frc.robot.constants;

public class RobotMap {

    public static class SafetyMap {
        public static final double kMaxSpeed = 6.0;
        public static final double kMaxRotation = 1.0;
        public static final double kMaxAcceleration = 1.0;
        public static final double kMaxAngularAcceleration = 1.0;
        public static final double kJoystickDeadband = 0.1;
        public static  double kMaxSpeedChange = 1;
        public static double kFollowerCommand = 6;

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

    // You can add more mappings for other subsystems like intake, shooter, etc.

    public static class VisionMap {
        public static final double ballRadius = 9; // cm ; 3.5 inches
        public static final double targetHeight = 98.25; // cm ; 38.7 inches
        public static final double cameraHeight = 40.64; // cm ; 16 inches
        public static final double cameraAngle = 0; // degrees


        public static class BackCam {
            public static final String description = "For finding Notes and going towards them";
            public static final String kname = "BackCamera";
            public static final String ip = "http://10.2.1.17:5801";
            public static final String BACK_CAMERA_NETWORK_TABLES_NAME = "limelight-april";
            public static final double APRILTAG_HEIGHT = 0.0;
            public static final double CAMERA_HEIGHT = 0.0;
            public static double horizontal_fov = 59.6;
            public static double vertical_fov = 45.7;
            public static int CameraWidth = 640;
            public static int CameraHeight = 480;
        }
    
        public static class FrontCam {
            public static final String description = "For finding AprilTags and for distance calculation";
            public static final String kName = "FrontCamera";
            public static final String ip = "http://10.2.1.43:5801";
            public static final String FRONT_CAMERA_NETWORK_TABLES_NAME = "limelight-notes";
            public static double horizontal_fov = 63.3;
            public static double vertical_fov = 49.7;
            public static int CameraWidth = 640;
            public static int CameraHeight = 480;

    }

    
}
public static class IntakeMap {
        public static final int INTAKE_MOTOR = 11;
        public static final int INTAKE_WRIST = 12;

    }
}
