package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HandoffToElevator;
import frc.robot.commands.elevator.ElevatorMoveLimit;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.IntakeUntilNoteIn;
import frc.robot.commands.intake.RotateWristBasic;
import frc.robot.commands.swerve.AimToBall;
import frc.robot.constants.RobotMap;
import frc.robot.constants.RobotMap.UsbMap;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.IntakeIRSensor;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.camera.Back_Camera;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;


public class RobotContainer {
 
    // Replace with CommandPS4Controller or CommandJoystick if needed
    SwerveSubsystem swerveSubsystem;
    IntakeWheels intakeWheels;
    Wrist wrist;
    Back_Camera backCamera;
    Elevator elevator;
    IntakeIRSensor intakeIRSensor;
    CommandXboxController driverController;
    CommandXboxController operatorController;
//     CommandSwerveDrivetrain drivetrain;


    public RobotContainer() {
        // Configure the trigger bindings

        driverController = new CommandXboxController(
                UsbMap.DRIVER_CONTROLLER
        );

        operatorController = new CommandXboxController(
                UsbMap.OPERATOR_CONTROLLER
        );

        swerveSubsystem = new SwerveSubsystem(
                Subsystems.SWERVE_DRIVE ,
                Subsystems.SWERVE_DRIVE.getNetworkTable() ,
                RobotMap.SensorMap.GYRO_PORT ,
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

        


        swerveSubsystem.setDefaultCommand(); 
//      swerveSubsystem.setBetaDefaultCommand(); /*RUN FODC INSTEAD OF DEFAULT COMMAND*/
        configureBindings();


    

    //! YOU DO NOT NEED THIS BELOW CODE SINCE RUNNING THE SWERVE DRIVETRAIN IN DEFAULT COMMAND WILL RUN THE SWERVE DRIVETRAIN
        //  drivetrain.setDefaultCommand(new ParallelCommandGroup(
        //         drivetrain.applyRequest(() -> DrivetrainConstants.drive
        //                 .withVelocityX(-driverController.getLeftY()
        //                         * SafetyMap.kMaxSpeed * SafetyMap.kMaxSpeedChange)
        //                 .withVelocityY(-driverController.getLeftX()
        //                         * SafetyMap.kMaxSpeed * SafetyMap.kMaxSpeedChange)
        //                 .withRotationalRate(-driverController.getRightX() 
        //                         * SafetyMap.kMaxSpeed * SafetyMap.kMaxSpeedChange)),
        //         new RepeatCommand(
        //                 new InstantCommand(this::printCurrentStickValues))));
    }


    private void configureBindings() { 
        driverController.start()
        .onTrue(swerveSubsystem.drivetrain.runOnce(swerveSubsystem.drivetrain::seedFieldRelative));
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
    public Object[] TestCommands() {
        return new Object[] {
            "Move Elevator Up", new ElevatorMoveLimit(),
            "Deply Intake ", new DeployIntake(wrist, intakeWheels, intakeIRSensor, driverController, driverController),
            "Intake Until Note in", new IntakeUntilNoteIn(intakeWheels, intakeIRSensor, driverController, operatorController),
            "Aim at Ball", new AimToBall(swerveSubsystem, backCamera),
            "Handoff ", new HandoffToElevator(wrist, intakeWheels, intakeIRSensor, driverController, operatorController)
        };
    }

}
