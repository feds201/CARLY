package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ComandCenter;
import frc.robot.constants.RobotMap.VisionMap;
import frc.robot.constants.RobotMap.VisionMap.CameraConfig;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.camera.Back_Camera;
import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.Smooth;

public class AimToBall extends Command {

    private PIDController rotationalPID;
    private PIDController strafePID;
    private PIDController distancePID;
    private CommandSwerveDrivetrain c_swerve;
    private Back_Camera c_visionsystem;
    private ShuffleboardTab tab;
    private Smooth X_filter;
    private Smooth Y_filter;
    private double X_smooth;
    private double Y_smooth;
    private double Output_linear;
    private double Output_strafe;
    private double Output_rotation;
    private BooleanSupplier isEnnabled;

    public AimToBall(CommandSwerveDrivetrain drivetrain, Back_Camera visionsystem) {
        c_swerve = drivetrain;
        c_visionsystem = visionsystem;
        X_filter = new Smooth(5);
        Y_filter = new Smooth(2);
        tab = ComandCenter.tab;
        // Initialize PID controllers
        distancePID = new PIDController(2, 0.0, 0.0);
        strafePID = new PIDController(0.085, 0.0, 0);
        rotationalPID = new PIDController(.1, 0.0, 0);

        rotationalPID.setTolerance(1);
        strafePID.setTolerance(0);
        distancePID.setTolerance(0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        isEnnabled = ()-> true;
        ComandCenter.addAimToBallCommand(isEnnabled);
        c_swerve.resetPID();
        rotationalPID.reset();
        strafePID.reset();
        distancePID.reset();

        tab.addNumber(this.getName()+"/Distance to Ball", () -> Y_smooth);
        tab.addNumber(this.getName()+"/Limelight Smoothed X", () -> X_smooth);
        tab.addNumber(this.getName()+"/Rotation Output", () -> Output_rotation);
        tab.addNumber(this.getName()+"/Strafe Output",() -> Output_strafe);
        tab.addNumber(this.getName()+"/X output", () -> Output_linear);
    }

    @Override
    public boolean isFinished() {
        return Y_smooth < 1;
    }

    @Override
    public void execute() {
        X_smooth = X_filter.calculate(CameraConfig.tx);
        Y_smooth = Y_filter.calculate(CameraConfig.distance);

        // Stop the command if the distance is less than 5
        if (Y_smooth < 1) {
            c_swerve.setControl(
            DrivetrainConstants.brake
            );
        }

        SmartDashboard.putNumber("Distance", Y_smooth);

        // Calculate PID outputs
        Output_linear = distancePID.calculate(Y_smooth);
        Output_strafe = strafePID.calculate(X_smooth);
        Output_rotation = (rotationalPID.calculate(X_smooth) - Output_strafe) - Output_strafe / 2; // basic

        // Send control to the swerve drivetrain
        c_swerve.setControl(DrivetrainConstants.drive
                .withVelocityX(Output_linear * 0.25)
                .withVelocityY(-Output_strafe * 0.25)
                .withRotationalRate(Output_rotation * 0.25));
    }

    @Override
    public void end(boolean interrupted) {
        isEnnabled = ()-> false;
       
    }
}