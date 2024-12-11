package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DeployIntake;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;


public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    SwerveSubsystem swerveSubsystem;
    IntakeWheels intakeWheels;
    CommandXboxController driverController = new CommandXboxController(RobotMap.UsbMap.DRIVER_CONTROLLER);

    public RobotContainer() {
        // Configure the trigger bindings
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
        

        configureBindings();
        swerveSubsystem.setDefaultCommand(); 
    //    swerveSubsystem.setBetaDefaultCommand();   

    }


    private void configureBindings() { }


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
}
