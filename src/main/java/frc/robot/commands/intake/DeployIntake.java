// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.subsystems.intake.IntakeIRSensor;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;

public class DeployIntake extends SequentialCommandGroup {
    public DeployIntake(Wrist wrist, IntakeWheels intakeWheels,
            IntakeIRSensor breakBeamSensorIntake, CommandXboxController driver, CommandXboxController operator) {
        addCommands(
                new ParallelDeadlineGroup(
                        new RotateWristToPosition(wrist, IntakeMap.WristPID.K_WRIST_FLOOR_POSITION),
                        new RunIntakeWheels(intakeWheels, () -> IntakeMap.K_INTAKE_NOTE_WHEEL_SPEED)),


                new IntakeUntilNoteIn(intakeWheels, breakBeamSensorIntake, driver, operator),
                new ResetIntake(wrist, intakeWheels, breakBeamSensorIntake, driver, operator));
    }
}
