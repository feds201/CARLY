package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotMap;
import frc.robot.constants.SwerveConstants;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;
import frc.robot.utils.DrivetrainConstants;

import java.util.Map;

public class SwerveSubsystem extends SubsystemABS {

    private ShuffleboardTab tab;
    private String tabName;
    private final Pigeon2 pigeonIMU;
    private CommandXboxController driverController;
    public CommandSwerveDrivetrain drivetrain;
    public SwerveSubsystem(Subsystems part, String tabName, int pigeonIMUID, CommandXboxController driverController) {
        super(part, tabName);
        this.tabName = tabName;
        this.pigeonIMU = new Pigeon2(pigeonIMUID); // Initialize Pigeon IMU
        this.driverController = driverController;
        this.drivetrain = DrivetrainConstants.drivetrain;
    }

    @Override
    public void init() {
        tab = getTab();
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        // Simulation-specific periodic tasks
    }

    public double applyDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0;
    }

    public double getRobotAngle() {
        return pigeonIMU.getAngle();
    }

    @Override
    public void setDefaultCommand() {
        DrivetrainConstants.drivetrain.setDefaultCommand(new GenericDrivetrain(driverController));
    }

    @Override
    public boolean isHealthy() {
        return true;
    }

    public void setBetaDefaultCommand() {
        DrivetrainConstants.drivetrain.setDefaultCommand(new FODC(driverController));
    }

    public void stop() {
        DrivetrainConstants.drivetrain.setControl(DrivetrainConstants.brake);
    }

    public void printcontroller() {
        try{
        tab.addNumber(tabName + "/Left Y", () -> applyDeadband(driverController.getLeftY(), 0.10));
        tab.addNumber(tabName + "/Left X", () -> applyDeadband(driverController.getLeftX(), 0.10));
        tab.addNumber(tabName + "/Right X", () -> applyDeadband(driverController.getRightX(), 0.10));
        tab.addNumber(tabName + "/Right Y", () -> applyDeadband(driverController.getRightY(), 0.10));
        tab.addNumber(tabName + "/Left Trigger", () -> applyDeadband(driverController.getLeftTriggerAxis(), 0.10));
        tab.addNumber(tabName + "/Right Trigger", () -> applyDeadband(driverController.getRightTriggerAxis(), 0.10));
        tab.addBoolean(tabName + "/Left Bumper", driverController.leftBumper());
        tab.addBoolean(tabName + "/Right Bumper", driverController.rightBumper());
        tab.addBoolean(tabName + "/A Button", driverController.a());
        tab.addBoolean(tabName + "/B Button", driverController.b());
        tab.addBoolean(tabName + "/X Button", driverController.x());
        tab.addBoolean(tabName + "/Y Button", driverController.y());
        tab.addBoolean(tabName + "/Start Button", driverController.start());
        tab.addBoolean(tabName + "/Back Button", driverController.back());
        } catch (Exception e) {
            
        }
    }

    public double snapToNearestLine(double angle, int i) {
        double snapAngle = 360.0 / RobotMap.SafetyMap.FODC.LineCount;
        return Math.round(angle / snapAngle) * snapAngle;
    }

    class GenericDrivetrain extends Command {

        private final CommandXboxController driverController;

        public GenericDrivetrain(CommandXboxController driverController) {
            addRequirements(DrivetrainConstants.drivetrain);
            this.driverController = driverController;
            printcontroller();
        }

        @Override
        public void execute() {
            super.execute();
            // Apply Deadband to the controller inputs
            double rightStickX = applyDeadband(driverController.getRightX(), 0.05);
            double rightStickY = applyDeadband(driverController.getRightY(), 0.05);
            double leftStickX = applyDeadband(driverController.getLeftX(), 0.05);

            new ParallelCommandGroup(
                    DrivetrainConstants.drivetrain.applyRequest(() -> DrivetrainConstants.drive
                            .withVelocityX(
                                    -rightStickY * RobotMap.SafetyMap.kMaxSpeed * RobotMap.SafetyMap.kMaxSpeedChange)
                            .withVelocityY(
                                    -rightStickX * RobotMap.SafetyMap.kMaxSpeed * RobotMap.SafetyMap.kMaxSpeedChange)
                            .withRotationalRate(leftStickX * SwerveConstants.MaxAngularRate)));
        }
    }

    class FODC extends Command {

        private final CommandXboxController driverController;
        private double angle;
        private double lastAngle = 0.0;
        private double snappedAngle;

        public FODC(CommandXboxController driverController) {
            addRequirements(DrivetrainConstants.drivetrain);
            this.driverController = driverController;
            printcontroller();
            tab.addNumber("Angle", () -> snappedAngle)
                    .withWidget(BuiltInWidgets.kGyro)
                    .withPosition(0, 0)
                    .withProperties(Map.of("majorTickSpacing", RobotMap.SafetyMap.FODC.LineCount, "startingAngle", 0));
            tab.addNumber("FODC/Angle", () -> angle)
                    .withWidget(BuiltInWidgets.kGyro)
                    .withPosition(0, 0)
                    .withProperties(Map.of("majorTickSpacing", RobotMap.SafetyMap.FODC.LineCount, "startingAngle", 0));
        }

        @Override
        public void execute() {
            super.execute();
            // Apply Deadband to the controller inputs
            double rightStickX = applyDeadband(driverController.getRightX(), 0.05);
            double rightStickY = applyDeadband(driverController.getRightY(), 0.05);
            double leftStickX = applyDeadband(driverController.getLeftX(), 0.05);
            double leftStickY = applyDeadband(driverController.getLeftY(), 0.05);
            double robotAngle;

            if (rightStickX != 0 || rightStickY != 0) {
                angle = Math.toDegrees(Math.atan2(rightStickY, rightStickX)) - 90; // Adjust angle by subtracting 90
                                                                                   // degrees
                lastAngle = angle; // Update last angle when joystick is moved
            } else {
                angle = lastAngle; // Use last angle when joystick is not moved
            }

            snappedAngle = snapToNearestLine(angle, RobotMap.SafetyMap.FODC.LineCount);
            robotAngle = getRobotAngle();
            RobotMap.SafetyMap.FODC.AngleDiff = snappedAngle - robotAngle;

            if (RobotMap.SafetyMap.FODC.AngleDiff > 180) {
                RobotMap.SafetyMap.FODC.AngleDiff -= 360;
            } else if (RobotMap.SafetyMap.FODC.AngleDiff < -180) {
                RobotMap.SafetyMap.FODC.AngleDiff += 360;
            }

            double output = DrivetrainConstants.drivetrain.getPIDRotation(RobotMap.SafetyMap.FODC.AngleDiff);

            DrivetrainConstants.drivetrain.setControl(DrivetrainConstants.drive
                    .withVelocityX(-leftStickY * SwerveConstants.MaxSpeed)
                    .withVelocityY(-leftStickX * SwerveConstants.MaxSpeed)
                    .withRotationalRate(output));
        }
    }

    @Override
    public void Failsafe() {
        stop();
    }
}
