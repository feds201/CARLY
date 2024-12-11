// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.intake.RunIntakeWheels;
// import frc.robot.commands.intake.RotateWristToPosition;
// import frc.robot.constants.IntakeConstants;
// import frc.robot.subsystems.intake.IntakeIRSensor;
// import frc.robot.subsystems.intake.IntakeWheels;
// import frc.robot.subsystems.intake.Wrist;


// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class HandoffToElevator extends SequentialCommandGroup {
//     private BooleanSupplier intakeBeamBroken;
//     public HandoffToElevator(Wrist wrist, IntakeWheels intakeWheels,
//                         IntakeIRSensor breakBeamSensorIntake, CommandXboxController driver, CommandXboxController operator) {
//                            intakeBeamBroken = breakBeamSensorIntake::getBeamBroken;
//         addCommands(
//             new ParallelDeadlineGroup(
//                 new RotateWristToPosition(wrist, IntakeConstants.WristPID.kWristHandoffPosition)
//                ),
//             new ParallelDeadlineGroup(
//                 new RunIntakeWheels(intakeWheels, () -> -IntakeConstants.kIntakeNoteWheelSpeed)

//                 //command not yet done!
//                ));
//     }
// }
