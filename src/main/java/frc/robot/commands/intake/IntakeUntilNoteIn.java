// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.subsystems.intake.IntakeIRSensor;
import frc.robot.subsystems.intake.IntakeWheels;


public class IntakeUntilNoteIn extends SequentialCommandGroup {
  public IntakeUntilNoteIn(IntakeWheels intakeWheels, IntakeIRSensor irSensor, CommandXboxController driver, CommandXboxController operator) {

    addCommands(
        new RunIntakeWheels(intakeWheels, () -> IntakeMap.K_INTAKE_NOTE_WHEEL_SPEED/2)
            .until(irSensor::getBeamBroken),
        new ParallelDeadlineGroup(
            new WaitCommand(IntakeMap.K_DISTANCE_SENSOR_DETECTED_DELAY),
            new RunIntakeWheels(intakeWheels, () -> IntakeMap.K_INTAKE_NOTE_WHEEL_SPEED/2)));
  }
}
