package frc.robot.subsystems.swerve;

import java.util.Map;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.FODC;
import frc.robot.commands.swerve.GenericDrivetrain;
import frc.robot.constants.RobotMap;
import frc.robot.constants.RobotMap.SafetyMap;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;
import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.FieldMap;

public class SwerveSubsystem extends SubsystemABS {

    private ShuffleboardTab tab;
    private String tabName;
    private final Pigeon2 pigeonIMU;
    private CommandXboxController driverController;
    public CommandSwerveDrivetrain drivetrain;
    private double robotFacingAngle;
    private SimpleWidget robotSpeedWidget;
    private FieldMap fieldMap;

    public SwerveSubsystem(Subsystems part, String tabName, int pigeonIMUID, CommandXboxController driverController) {
        super(part, tabName);
        this.tabName = tabName;
        this.pigeonIMU = new Pigeon2(pigeonIMUID); // Initialize Pigeon IMU
        this.driverController = driverController;
        this.drivetrain = DrivetrainConstants.drivetrain;
        this.fieldMap = new FieldMap();
        printcontroller();
    }

    @Override
    public void init() {
        tab = getTab();
        
         tab.add("Gyro",  robotFacingAngle)
                .withWidget(BuiltInWidgets.kGyro) // Use Gyro widget
                .withProperties(Map.of("startingAngle", 0, "majorTickSpacing", 45)) // Customize properties
                .withPosition(3, 0)
                .withSize(3, 3);

        robotSpeedWidget = tab.add("Robot Speed (%)", (RobotMap.SafetyMap.kMaxSpeedChange*100))
                .withPosition(3,5)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min",0,"max",100))
                .withSize(6,2);

    }

    @Override
    public void periodic() {
        robotFacingAngle = getRobotAngle();
        RobotMap.SafetyMap.kMaxSpeedChange = robotSpeedWidget.getEntry().getDouble(RobotMap.SafetyMap.kMaxSpeedChange * 100.0) / 100.0; //Get Slider Value
        Pose2d currentPose = drivetrain.getState().Pose;
        if (currentPose != null) {  // Add this null check
            fieldMap.updateRobotPose(currentPose);
        } else {
            // Handle the null pose.  Print a warning or take other action
            System.err.println("SwerveSubsystem.periodic(): drivetrain pose is null!");
        }
    }

    @Override
    public void simulationPeriodic() {
        robotFacingAngle = 0;
        RobotMap.SafetyMap.kMaxSpeedChange = robotSpeedWidget.getEntry().getDouble(RobotMap.SafetyMap.kMaxSpeedChange * 100.0) / 100.0; //Get Slider Value
        Pose2d currentPose = new Pose2d(0,1,new Rotation2d(40)); // This pose needs to be valid
        fieldMap.updateRobotPose(currentPose);
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
