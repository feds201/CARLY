package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.subsystems.intake.IntakeIRSensor;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.Wrist;

public class ResetIntake extends SequentialCommandGroup {
    public ResetIntake(Wrist wrist, IntakeWheels intakeWheels,
            IntakeIRSensor breakBeamSensorIntake, CommandXboxController driver, CommandXboxController operator) {
        addCommands(
              
                new ParallelDeadlineGroup(
                        new RotateWristToPosition(wrist, IntakeMap.WristPID.K_WRIST_SHOOTER_FEEDER_SETPOINT),
                        new RunIntakeWheels(intakeWheels, () -> 0)),
                        new ParallelDeadlineGroup(new WaitCommand(0.5), new RunIntakeWheels(intakeWheels, ()->0.1))
                        );
    }
}
