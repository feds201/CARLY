// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.RobotMap.SafetyMap;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.DrivetrainConstants;


public class GenericDrivetrain extends Command {

        private final CommandXboxController driverController;
        private final SwerveSubsystem swerveSubsystem;
        public GenericDrivetrain(CommandXboxController driverController, SwerveSubsystem swerveSubsystem) {
            addRequirements(DrivetrainConstants.drivetrain);
            this.driverController = driverController;
            this.swerveSubsystem = swerveSubsystem;
            swerveSubsystem.printcontroller();
        }

        @Override
        public void execute() {
            super.execute();
            // Apply Deadband to the controller inputs
            double rightStickX = swerveSubsystem.applyDeadband(driverController.getRightX(), 0.05);
            double rightStickY = swerveSubsystem.applyDeadband(driverController.getRightY(), 0.05);
            double leftStickX = swerveSubsystem.applyDeadband(driverController.getLeftX(), 0.05);

            new ParallelCommandGroup(
                    DrivetrainConstants.drivetrain.applyRequest(() -> DrivetrainConstants.drive
                            .withVelocityX(
                                    -rightStickY * SafetyMap.kMaxSpeed * SafetyMap.kMaxSpeedChange)
                            .withVelocityY(
                                    -rightStickX * SafetyMap.kMaxSpeed * SafetyMap.kMaxSpeedChange)
                            .withRotationalRate(leftStickX * SwerveConstants.MaxAngularRate)));
        }

     
    }

 