// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.SubsystemABS;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed

    SwerveSubsystem swerveSubsystem;
    CommandXboxController driverController = new CommandXboxController(RobotMap.UsbMap.DRIVER_CONTROLLER);

    public RobotContainer() {
        // Configure the trigger bindings
        swerveSubsystem = new SwerveSubsystem(
                Subsystems.SWERVE_DRIVE ,
                Subsystems.SWERVE_DRIVE.getNetworkTable() ,
                RobotMap.SensorMap.GYRO_PORT ,
                driverController
        );



        configureBindings();
        swerveSubsystem.setDefaultCommand(); /*       swerveSubsystem.setBetaDefaultCommand();    */

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
        };
    }
}
