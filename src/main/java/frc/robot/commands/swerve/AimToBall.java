package frc.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
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

    private Smooth X_filter;
    private Smooth Y_filter;

    public AimToBall(CommandSwerveDrivetrain drivetrain, Back_Camera visionsystem) {
        c_swerve = drivetrain;
        c_visionsystem = visionsystem;
        X_filter = new Smooth(5);
        Y_filter = new Smooth(2);

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
        SmartDashboard.putBoolean("AimToBallCommand", true);
        c_swerve.resetPID();
        rotationalPID.reset();
        strafePID.reset();
        distancePID.reset();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        double X_smooth = X_filter.calculate(c_visionsystem.getObject().getX());
        double Y_smooth = Y_filter.calculate(c_visionsystem.getObject().getDistance());
        // Calculate PID outputs

        double Output_linear = distancePID.calculate(Y_smooth);
        double Output_strafe = strafePID.calculate(X_smooth);
        double Output_rotation = (rotationalPID.calculate(X_smooth) - Output_strafe) - Output_strafe / 2; // basic

    
        // Log values to SmartDashboard

        SmartDashboard.putNumber("Limelight Smoothed X", X_smooth);
        SmartDashboard.putNumber("Distance to Ball", Y_smooth);
        SmartDashboard.putNumber("Rotation Output", Output_rotation);
        SmartDashboard.putNumber("Strafe Output", Output_strafe);
        SmartDashboard.putNumber("X output", Output_linear);

        // Send control to the swerve drivetrain
        c_swerve.setControl(DrivetrainConstants.drive
                .withVelocityX(Output_linear)
                .withVelocityY(-Output_strafe)
                .withRotationalRate(Output_rotation));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("AimToBallCommand", false);
    }
}
