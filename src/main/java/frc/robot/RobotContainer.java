package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DeployIntake;
import frc.robot.constants.RobotMap;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.intake.IntakeIRSensor;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;
import frc.robot.utils.Telemetry;


public class RobotContainer {
CommandSwerveDrivetrain drivetrain  = TunerConstants.DriveTrain;

 private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

    public double swerveSpeedMultiplier = 1;    
    // Replace with CommandPS4Controller or CommandJoystick if needed
    SwerveSubsystem swerveSubsystem;
    IntakeWheels intakeWheels;
    Wrist wrist;
    IntakeIRSensor intakeIRSensor;
    CommandXboxController driverController = new CommandXboxController(RobotMap.UsbMap.DRIVER_CONTROLLER);

    public RobotContainer() {
        // Configure the trigger bindings
        swerveSubsystem = new SwerveSubsystem(
                Subsystems.SWERVE_DRIVE ,
                Subsystems.SWERVE_DRIVE.getNetworkTable() ,
                RobotMap.SensorMap.GYRO_PORT ,
                driverController
        );


        //Intake Subsystem
        intakeWheels = new IntakeWheels(Subsystems.INTAKE, Subsystems.INTAKE.getNetworkTable());
        wrist = new Wrist(Subsystems.INTAKE, Subsystems.INTAKE.getNetworkTable());
        intakeIRSensor = new IntakeIRSensor(Subsystems.INTAKE, Subsystems.INTAKE.getNetworkTable());

        configureBindings();
        // drivetrain.setDefaultCommand(); 
    //    swerveSubsystem.setBetaDefaultCommand();   
         drivetrain.setDefaultCommand(new ParallelCommandGroup(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driverController.getLeftY()
                                * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                        .withVelocityY(-driverController.getLeftX()
                                * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
                        .withRotationalRate(-driverController.getRightX() *
                                SwerveConstants.MaxAngularRate * swerveSpeedMultiplier)),
                new RepeatCommand(
                        new InstantCommand(this::printCurrentStickValues))));
    }


    private void configureBindings() { 
        driverController.start()
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));
        /*DO NOT UN-COMMENT THIS LINE UNTIL INTAKE CONSTANTS HAVE BEEN FIXED!!! */
        // driverController.a().whileTrue(new DeployIntake(wrist, intakeWheels, intakeIRSensor, driverController, driverController));
    }


    public Command getAutonomousCommand() {return null; }


//     DO NOT REMOVE
    public SubsystemABS[] SafeGuardSystems() {
        return new SubsystemABS[] {
                swerveSubsystem ,

        };
    }
    public Object[] TestCommands() {
        return new Object[] {
            new DeployIntake(null, null, null, driverController, driverController)
        };
    }

    private void printCurrentStickValues() {
        SmartDashboard.putNumber("left y",
                -driverController.getLeftY()
                        * SwerveConstants.MaxSpeed * swerveSpeedMultiplier);
        SmartDashboard.putNumber("left x",
                -driverController.getLeftX()
                        * SwerveConstants.MaxSpeed * swerveSpeedMultiplier);
        SmartDashboard.putNumber("right x",
                -driverController.getRightX() *
                        SwerveConstants.MaxAngularRate * swerveSpeedMultiplier);
    }
}
