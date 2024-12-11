package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

public class RobotMap {

    public static class SafetyMap {
        public static final double kMaxSpeed = 6.0;
        public static final double kMaxRotation = 1.0;
        public static final double kMaxAcceleration = 1.0;
        public static final double kMaxAngularAcceleration = 1.0;
        public static final double kJoystickDeadband = 0.1;
        public static  double kMaxSpeedChange = 1;
        public static double kFollowerCommand = 6;

        public static class FODC {
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
        public static final int ELEVATOR_SPEED = 0;
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


        public static class CameraConfig {
            public static class BackCam {
                public static final int CAMERA_HEIGHT = 480;
                public static final int CAMERA_WIDTH = 640;
                public static final double TARGET_HEIGHT = 0.0;
                public static final double HORIZONTAL_FOV = 59.6;
                public static final double VERTICAL_FOV = 45.7;
            }
        
            public static class FrontCam {
                public static final int CAMERA_HEIGHT = 480;
                public static final int CAMERA_WIDTH = 640;
                public static final double TARGET_HEIGHT = 0.0;
                public static final double HORIZONTAL_FOV = 59.6;
                public static final double VERTICAL_FOV = 45.7;
            }
        }
        

    
}


public static class IntakeMap {
    public static final int INTAKE_MOTOR = 11;
    public static final int INTAKE_WRIST = 12;
    public static final double K_INTAKE_NOTE_WHEEL_SPEED = -1;
    public static final double K_SPIT_OUT_NOTE_WHEEL_SPEED = 1;
    public static final double K_AMP_IN_WHEEL_SPEED = 1;
    public static final double K_HANDOFF_NOTE_WHEEL_SPEED = 0.6;
    public static final double K_DISTANCE_SENSOR_DETECTED_DELAY = 0.35;

    public static class SensorConstants {
        public static final int INTAKE_BB_RECEIVER = 6;
        public static final int INTAKE_BB_TRANSMITTER = 7;
        public static final int INTAKE_ROTATE_ENCODER = 2;
    }

    public static class WristPID {
        public static final double KP = 0.003;
        public static final double KI = 0.001;
        public static final double KD = 0.0;
        public static final double KIZONE = Double.POSITIVE_INFINITY;
        public static final double K_ROTATION_TOLERANCE = 10;

        /* TODO: Change these values after testing with Carly Intake */
        public static final double K_SPIT_OUT_POSITION = 201;
        public static final double K_WRIST_FLOOR_POSITION = 308;
        public static final double K_WRIST_HANDOFF_POSITION = 308;
        public static final double K_WRIST_SHOOTER_FEEDER_SETPOINT = 94;
        public static double K_WRIST_OUT_OF_THE_WAY = 150;

         // TODO: Ideally all of the above positions should be based on this "home" position so we only have to change this

        public static PIDController getWristPID() {
            PIDController pid = new PIDController(KP, KI, KD);
            pid.setIZone(KIZONE);
            pid.setTolerance(K_ROTATION_TOLERANCE);
            return pid;
        }

    }


}
}