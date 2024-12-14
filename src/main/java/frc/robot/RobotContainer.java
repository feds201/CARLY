package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.elevator.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.swerve.AimToBall;
import frc.robot.commands.swerve.GenericDrivetrain;
import frc.robot.constants.RobotMap.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.camera.Back_Camera;
import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;


public class RobotContainer {

    private SwerveSubsystem swerveSubsystem;
    private IntakeWheels intakeWheels;
    private Wrist wrist;
    private Back_Camera backCamera;
    private Elevator elevator;
    private IntakeIRSensor intakeIRSensor;
    private CommandXboxController driverController;
    private CommandXboxController operatorController;


    public RobotContainer() {

        driverController = UsbMap.driverController;
        operatorController = UsbMap.operatorController;

        swerveSubsystem = new SwerveSubsystem(
                Subsystems.SWERVE_DRIVE ,
                Subsystems.SWERVE_DRIVE.getNetworkTable() ,
                SensorMap.GYRO_PORT ,
                driverController
        );

        intakeWheels = new IntakeWheels(
                Subsystems.INTAKE, 
                Subsystems.INTAKE.getNetworkTable()
        );

        wrist = new Wrist(
                Subsystems.INTAKE, 
                Subsystems.INTAKE.getNetworkTable()
        );

        intakeIRSensor = new IntakeIRSensor(
                Subsystems.INTAKE, 
                Subsystems.INTAKE.getNetworkTable()
        );


        backCamera = new Back_Camera(
                Subsystems.VISION, 
                Subsystems.VISION.getNetworkTable()
        );

        elevator = new Elevator(
                Subsystems.ELEVATOR, 
                Subsystems.ELEVATOR.getNetworkTable()
        );

        
        DrivetrainConstants.drivetrain.setDefaultCommand(new GenericDrivetrain(driverController));

        configureBindings();



    }


    private void configureBindings() { 
        driverController.start()
        .onTrue(DrivetrainConstants.drivetrain.runOnce(DrivetrainConstants.drivetrain::seedFieldRelative));
        driverController.a().whileTrue(new RotateWristBasic(wrist, ()-> -0.5));
        driverController.b().whileTrue(new RunIntakeWheels(intakeWheels, ()-> 0.5));
        driverController.leftTrigger().whileTrue(new AimToBall(DrivetrainConstants.drivetrain, backCamera));

        
        /*DO NOT UN-COMMENT THIS LINE UNTIL INTAKE CONSTANTS HAVE BEEN FIXED!!! */
        // driverController.a().whileTrue(new DeployIntake(wrist, intakeWheels, intakeIRSensor, driverController, driverController));
    }


    public Command getAutonomousCommand() {return null; }


//     DO NOT REMOVE
    public SubsystemABS[] SafeGuardSystems() {
        return new SubsystemABS[] {
                swerveSubsystem ,
                intakeWheels ,
                wrist ,
                backCamera ,
                elevator ,
                intakeIRSensor
        };
    }

    // ONLY RUNS IN TEST MODE
    public Object[] TestCommands() {
        return new Object[] {
            "Move Elevator Up", new ElevatorMoveLimit(),
            "Deply Intake ", new DeployIntake(wrist, intakeWheels, intakeIRSensor, driverController, operatorController),
            "Intake Until Note in", new IntakeUntilNoteIn(intakeWheels, intakeIRSensor, driverController, operatorController),
            "Aim at Ball", new AimToBall(DrivetrainConstants.drivetrain, backCamera),
            "Handoff ", new HandoffToElevator(wrist, intakeWheels, intakeIRSensor, driverController, operatorController),
            "Test Wrist", new RotateWristBasic(wrist, ()-> -0.5),
            "Test Intake Wheels", new RunIntakeWheels(intakeWheels, ()-> 0.5)
        };
    }

}
