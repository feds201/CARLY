
// package frc.robot.utils;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import frc.robot.constants.RobotMap;

// public class VisionObject {
//     static NetworkTable table = NetworkTableInstance.getDefault().getTable(RobotMap.VisionMap.BackCam.kname);
//     private double x;
//     private double y;
//     private double area;
//     private ObjectType type;


//     public VisionObject(double x, double y, double area, ObjectType type) {
//         this.x = x;
//         this.y = y;
//         this.area = area;
//         this.type = type;
//     }

//     public void SetNTEntry(NetworkTable tableInstance) {
//         NetworkTableEntry txEntry = tableInstance.getEntry("tx");
//         NetworkTableEntry tyEntry = tableInstance.getEntry("ty");
//         NetworkTableEntry taEntry = tableInstance.getEntry("ta");


//         // Initialize the values
//         this.x = txEntry.getDouble(0);
//         this.y = tyEntry.getDouble(0);
//         this.area = taEntry.getDouble(0);
//     }

//     public double getX() {
//         return x;
//     }

//     public static boolean isPresent(){
//         return table.getEntry("tv").getDouble(0) == 1.0;
//     }

//     public void setX(double x) {
//         this.x = x;
//     }

//     public double getY() {
//         return y;
//     }

//     public void setY(double y) {
//         this.y = y;
//     }

//     public double getArea() {
//         return area;
//     }

//     public void setArea(double area) {
//         this.area = area;
//     }

//     public void update(double x, double y, double area) {
//         this.x = x;
//         this.y = y;
//         this.area = area;
//     }

//     public ObjectType getType() {
//         return type;
//     }

//     public void setType(ObjectType type) {
//         this.type = type;
//     }

//     public double[] getAngle() {
//         switch (type) {
//             case APRIL_TAG:
//                 double[] angle = new double[2];
//                 double nx = ((double) 1 / ((double)RobotMap.VisionMap.B.CameraWidth/ 2)) * (x - ((double) RobotMap.VisionMap.BackCamera.CameraWidth / 2 - 0.5));
//                 double ny = ((double) 1 / ((double) RobotMap.VisionMap.BackCamera.CameraHeight / 2)) * (y - ((double) RobotMap.VisionMap.BackCamera.CameraWidth / 2 - 0.5));
//                 double vpw = 2.0 * Math.tan(RobotMap.VisionMap.BackCamera.HorizontalFOV / 2);
//                 double vph = 2.0 * Math.tan(RobotMap.VisionMap.BackCamera.VerticalFOV / 2);
//                 double x = vpw / 2 * nx;
//                 double y = vph / 2 * ny;
//                 double ax = Math.atan2(1, x);
//                 double ay = Math.atan2(1, y);
//                 angle[0] = Math.toDegrees(ax);
//                 angle[1] = Math.toDegrees(ay);
//                 return angle;
//             case INFINITE_CHARGE_BALLS:
//                 double[] angle1 = new double[2];
//                 double nx1 = ((double) 1 / ((double) RobotMap.VisionMap.BackCamera.CameraWidth / 2)) * (this.x - ((double) RobotMap.VisionMap.BackCamera.CameraWidth / 2 - 0.5));
//                 double ny1 = ((double) 1 / ((double) RobotMap.VisionMap.BackCamera.CameraHeight / 2)) * (this.y - ((double) RobotMap.VisionMap.BackCamera.CameraHeight / 2 - 0.5));
//                 double vpw1 = 2.0 * Math.tan(RobotMap.VisionMap.BackCamera.HorizontalFOV / 2);
//                 double vph1 = 2.0 * Math.tan(RobotMap.VisionMap.BackCamera.HorizontalFOV / 2);
//                 double x1 = vpw1 / 2 * nx1;
//                 double y1 = vph1 / 2 * ny1;
//                 double ax1 = Math.atan2(1, x1);
//                 double ay1 = Math.atan2(1, y1);
//                 angle1[0] = Math.toDegrees(ax1);
//                 angle1[1] = Math.toDegrees(ay1);
//                 return angle1;
//             default:
//                 throw new IllegalArgumentException("Invalid type");
//         }
//     }

//     public double getYaw() {
//         return switch (type) {
//             case APRIL_TAG -> getAngle()[0];
//             case INFINITE_CHARGE_BALLS -> getAngle()[0];
//             default -> throw new IllegalArgumentException("Invalid type");
//         };
//     }

//     public double getPitch() {
//         return switch (type) {
//             case APRIL_TAG  -> getAngle()[1];
//             case INFINITE_CHARGE_BALLS -> getAngle()[1];
//             default -> throw new IllegalArgumentException("Invalid type");
//         };
//     }

//     public double getDistance() {
//         switch (type) {
//             case APRIL_TAG:
//                 throw new IllegalArgumentException("Finish the code first before doing something stupid");
//             case INFINITE_CHARGE_BALLS:
//                 double targetOffsetAngle_Vertical = y;
//                 double limelightMountAngleDegrees = RobotMap.VisionMap.cameraAngle;
//                 double limelightLensHeightMetres = RobotMap.VisionMap.cameraHeight;
//                 double goalHeightMetres = RobotMap.VisionMap.targetHeight;
//                 double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
//                 double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
//                 return (goalHeightMetres - limelightLensHeightMetres) / Math.tan(angleToGoalRadians);
//             default:
//                 throw new IllegalStateException("Unexpected value: " + type);
//         }
//     }



// }