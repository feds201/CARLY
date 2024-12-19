package frc.robot;

import java.util.Map;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    private SendableChooser<Command> autonChooser = new SendableChooser<>();


    public RobotContainer() {

        double swerveSpeedMultiplier = 0.4;
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

        
        // DrivetrainConstants.drivetrain.setDefaultCommand(new GenericDrivetrain(driverController));
        DrivetrainConstants.drivetrain.setDefaultCommand(

        DrivetrainConstants.drivetrain.applyRequest(() -> DrivetrainConstants.drive
        .withVelocityX(-driverController.getLeftY() * SafetyMap.kMaxSpeed * SafetyMap.kMaxSpeedChange *swerveSpeedMultiplier)
        .withVelocityY(-driverController.getLeftX() * SafetyMap.kMaxSpeed * SafetyMap.kMaxSpeedChange *swerveSpeedMultiplier)
        .withRotationalRate(-driverController.getRightX() * SafetyMap.kMaxAcceleration * SafetyMap.kAngularRateMultiplier))
        );

        setupNamedCommands();
        setupPaths();
        configureBindings();



    }


    private void configureBindings() { 
        driverController.start()
            .onTrue(DrivetrainConstants.drivetrain.runOnce(DrivetrainConstants.drivetrain::seedFieldRelative));

        // driverController.b()
        //     .whileTrue(new RunIntakeWheels(intakeWheels, ()-> 0.25));

        driverController.leftBumper()
            .whileTrue(new MoveElevator(elevator, 0.15))
            .onFalse(new MoveElevator(elevator, 0.00));

        driverController.rightBumper()
            .whileTrue(new MoveElevator(elevator, -0.2))
            .onFalse(new MoveElevator(elevator, 0.00));

        driverController.x()
            .onFalse(new MoveElevator(elevator, 0.00));

        driverController.rightTrigger()
            .onTrue(new DeployIntake(wrist, intakeWheels, intakeIRSensor, driverController, driverController))
            .onFalse(new ResetIntake(wrist, intakeWheels, intakeIRSensor, driverController, driverController));

        driverController.leftTrigger()
            .onTrue(new RotateWristToPosition(wrist, IntakeMap.WristPID.K_WRIST_FLOOR_POSITION))
            .onFalse(new RotateWristToPosition(wrist, IntakeMap.WristPID.K_WRIST_SHOOTER_FEEDER_SETPOINT));

        // driverController.leftBumper()
            // .onTrue(new RotateWristToPositionInfinite(wrist, IntakeMap.WristPID.K_WRIST_HANDOFF_POSITION));

        // driverController.y()
            // .onTrue(new AimToBall(DrivetrainConstants.drivetrain, backCamera));
    }

    
    private void setupNamedCommands() {
        NamedCommands.registerCommand(
            "Rotate Intake Wheels", 
            new RunIntakeWheels(intakeWheels, ()-> 0.5)
        );
        NamedCommands.registerCommand(
            "Rotate Wrist", 
            new RotateWristBasic(wrist, ()-> -0.5)
        );
        NamedCommands.registerCommand(
            "Deploy Intake", 
            new DeployIntake(wrist, intakeWheels, intakeIRSensor, driverController, operatorController)
        );
        NamedCommands.registerCommand(
            "Intake Until Note In", 
            new IntakeUntilNoteIn(intakeWheels, intakeIRSensor, driverController, operatorController)
        );
        NamedCommands.registerCommand(
            "Aim at Ball", 
            new AimToBall(DrivetrainConstants.drivetrain, backCamera)
        );
        NamedCommands.registerCommand(
            "Handoff", 
            new HandoffToElevator(wrist, intakeWheels, intakeIRSensor, driverController, operatorController)
        );
        NamedCommands.registerCommand(
            "Move Elevator Up", 
            new ElevatorMoveLimit()
        );
        NamedCommands.registerCommand(
            "Field Relative", 
            DrivetrainConstants.drivetrain.runOnce(DrivetrainConstants.drivetrain::seedFieldRelative)
        );

    }

    public void setupPaths() {
        autonChooser.setDefaultOption("Do Nothing", null);
        autonChooser.addOption("Follow Path", DrivetrainConstants.drivetrain.getAutoPath("Test Path"));
        autonChooser.addOption("Rotate Intake Wheels", NamedCommands.getCommand("Rotate Intake Wheels"));
        autonChooser.addOption("Rotate Wrist", NamedCommands.getCommand("Rotate Wrist"));
        autonChooser.addOption("Deploy Intake", NamedCommands.getCommand("Deploy Intake"));
        autonChooser.addOption("Intake Until Note In", NamedCommands.getCommand("Intake Until Note In"));
        autonChooser.addOption("Aim at Ball", NamedCommands.getCommand("Aim at Ball"));
        autonChooser.addOption("Handoff", NamedCommands.getCommand("Handoff"));
        autonChooser.addOption("Move Elevator Up", NamedCommands.getCommand("Move Elevator Up"));
        autonChooser.addOption("Field Relative", NamedCommands.getCommand("Field Relative"));
        Shuffleboard.getTab(Subsystems.LIVEWINDOW.getNetworkTable()).add("Auton Chooser",autonChooser) .withSize(2, 1).withProperties(Map.of("position", "0, 0"));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    // DO NOT REMOVE
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

    public Object[] TestAutonCommands() {
        return new Object[] {
            "Follow Path", DrivetrainConstants.drivetrain.getAutoPath("Test Path"),
            "Follow Path 2", DrivetrainConstants.drivetrain.getAutoPath("Test Path 2"),
        };
    }

}
