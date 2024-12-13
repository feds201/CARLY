package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.FODC;
import frc.robot.commands.swerve.GenericDrivetrain;
import frc.robot.constants.RobotMap.SafetyMap;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;
import frc.robot.utils.DrivetrainConstants;

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
        DrivetrainConstants.drivetrain.setDefaultCommand(new GenericDrivetrain(driverController, this));
    }

    @Override
    public boolean isHealthy() {
        return true;
    }

    public void setBetaDefaultCommand() {
        DrivetrainConstants.drivetrain.setDefaultCommand(new FODC(driverController, this));
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
        double snapAngle = 360.0 / SafetyMap.FODC.LineCount;
        return Math.round(angle / snapAngle) * snapAngle;
    }


    @Override
    public void Failsafe() {
        stop();
    }
}
