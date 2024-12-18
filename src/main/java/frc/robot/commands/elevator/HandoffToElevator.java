package frc.robot.commands.elevator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.RunIntakeWheels;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.commands.intake.RotateWristToPosition;
import frc.robot.subsystems.intake.IntakeIRSensor;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;

public class HandoffToElevator extends SequentialCommandGroup {
    private BooleanSupplier intakeBeamBroken;

    public HandoffToElevator(
        Wrist wrist, 
        IntakeWheels intakeWheels,
        IntakeIRSensor breakBeamSensorIntake, 
        CommandXboxController driver, 
        CommandXboxController operator) 
        {
        intakeBeamBroken = breakBeamSensorIntake::getBeamBroken;

        addCommands(
            new ParallelDeadlineGroup(
                new RotateWristToPosition(wrist, IntakeMap.WristPID.K_WRIST_HANDOFF_POSITION)
            ),
            new ParallelDeadlineGroup(
                new RunIntakeWheels(intakeWheels, () -> -IntakeMap.K_INTAKE_NOTE_WHEEL_SPEED)
            )

            // Not Yet Implemented
        );
    }
}
