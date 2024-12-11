package frc.robot.subsystems.vision.camera;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.vision.VisionABC;
import frc.robot.subsystems.vision.VisionVariables;
import frc.robot.subsystems.vision.utils.VisionObject;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ObjectType;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

import java.util.Random;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class Back_Camera extends VisionABC {
		public static String nt_key;
		private static final ShuffleboardTab tab = Shuffleboard.getTab("FrontCamera");
		public static NetworkTable table;
		public static VisionObject tagCenter;

		public Random random = new Random();
		public Back_Camera() {
				nt_key = RobotMap.VisionMap.BackCam.BACK_CAMERA_NETWORK_TABLES_NAME;
				table = NetworkTableInstance.getDefault().getTable(nt_key);
				tagCenter = new VisionObject(0, 0, 0, ObjectType.APRIL_TAG);
		}

		private final static Back_Camera INSTANCE = new Back_Camera();

		@SuppressWarnings("WeakerAccess")
		public static Back_Camera getInstance() {
				return INSTANCE;
		}

		@Override
		public void simulationPeriodic() {
				tagCenter.update(
						random.nextDouble() * 100,
						random.nextDouble() * 100,
						random.nextDouble() * 360
				);
				Periodic();
		}
		@Override
		public void periodic() {
				tagCenter.update(
						table.getEntry("tx").getNumber(0).doubleValue(),
						table.getEntry("ty").getNumber(0).doubleValue(),
						table.getEntry("ts").getNumber(0).doubleValue()
				);
				Periodic();
		}


		@Override
		public boolean CheckTarget() {
				return table.getEntry("tv").getNumber(0).intValue() == 1;
		}

		@Override
		public Translation2d GetTarget(VisionObject object) {
				return new Translation2d(0, 0);
		}

		@Override
		public void setPipeline(int pipeline) {
				LimelightHelpers.setPipelineIndex(nt_key,pipeline);
		}

		@Override
		public void setLEDMode(int mode) {
				switch ( mode ) {
						case 0:
								LimelightHelpers.setLEDMode_ForceOff(nt_key);
								break;
						case 1:
								LimelightHelpers.setLEDMode_ForceOn(nt_key);
								break;
						case 2:
								LimelightHelpers.setLEDMode_ForceBlink(nt_key);
								break;
						default:
								break;
				}
		}

		/**
		 * @param mode 0: Driver Camera Mode, 1: Processor Camera Mode
		 */
		@Override
		public void setCamMode(int mode){
				switch ( mode ) {
						case 0:
								LimelightHelpers.setCameraMode_Driver(nt_key);
								break;
						case 1:
								LimelightHelpers.setCameraMode_Processor(nt_key);
								break;
						default:
								break;
				}
		}

		@Override
		public Command BlinkLED() {
				return runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink(nt_key))
						       .andThen(waitSeconds(2))
						       .andThen(() -> LimelightHelpers.setLEDMode_ForceOff(nt_key));
		}

		@Override
		public Command TurnOffLED() {
				return runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(nt_key));
		}

		private void Periodic(){
				SmartDashboard.putBoolean("Target Found", CheckTarget());
				VisionVariables.BackCam.tv = table.getEntry("tv").getNumber(0).intValue();
				VisionVariables.ExportedVariables.Distance = tagCenter.getDistance();
				VisionVariables.BackCam.CameraMode = table.getEntry("camMode").getNumber(0).intValue();
				VisionVariables.BackCam.LEDMode = table.getEntry("ledMode").getNumber(0).intValue();
				VisionVariables.BackCam.target = tagCenter;
				SmartDashboard.putNumber("Distance", tagCenter.getDistance());
				SmartDashboard.putNumber("X", tagCenter.getX());
				SmartDashboard.putNumber("Y", tagCenter.getY());

		}


}

