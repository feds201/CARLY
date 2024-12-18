// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;


public class Wrist extends SubsystemABS {
  private CANSparkMax wristRotation;
  private DutyCycleEncoder wristRotationEncoder;

  private PIDController pid = IntakeMap.WristPID.getWristPID();
 

  private DoubleEntry wristVoltage;
  private DoubleEntry rotationEncoderValue;
  private DoubleEntry rotationAngle;
  private DoubleEntry rotationTarget;

  private BooleanEntry failure;
  private BooleanEntry towardShooter;
  private ShuffleboardTab tab;
  private double rawEncoderValue;
  private double rotationAngleValue;


  public Wrist(Subsystems part, String tabName) {
    super(part, tabName);
    tab = getTab();

    wristVoltage = ntTable.getDoubleTopic("wrist_voltage").getEntry(0);
    rotationEncoderValue = ntTable.getDoubleTopic("rotation_value").getEntry(0);
    rotationAngle = ntTable.getDoubleTopic("rotation_angle").getEntry(0);
    rotationTarget = ntTable.getDoubleTopic("rotation_target").getEntry(0);
    failure = ntTable.getBooleanTopic("failure").getEntry(false);
    towardShooter = ntTable.getBooleanTopic("toward_shooter").getEntry(false);

    wristRotationEncoder.setPositionOffset(0);
     rawEncoderValue = wristRotationEncoder.get();
    rotationAngleValue = rawEncoderValue * 360;

    tab.addNumber("Wrist Encoder Value", () -> rawEncoderValue);
    tab.addNumber("Wrist Angle Raw (enc * 360)",() ->  rawEncoderValue * 360);
    tab.addNumber("Wrist Abs Position/ Position",() -> wristRotationEncoder.getAbsolutePosition());
    tab.addNumber("Wrist Abs Position/ Offset",() ->  wristRotationEncoder.getAbsolutePosition() - wristRotationEncoder.getPositionOffset());
    tab.addNumber("Wrist Angle Position/ Position",() -> wristRotationEncoder.getAbsolutePosition() * 360);
    tab.addNumber("Wrist Angle Position/ Offset",() ->  (wristRotationEncoder.getAbsolutePosition() - wristRotationEncoder.getPositionOffset()) * 360);
    tab.addNumber("Wrist after wrap around check",() -> rotationAngleValue);
    if (rawEncoderValue < 0) {
      tab.addNumber("Encoder negative", () -> rawEncoderValue);
    } else {
      tab.addNumber("Encoder positive",() ->  rawEncoderValue + 360);
    }


  }


  @Override
  public void init() {
    wristRotation = new CANSparkMax(IntakeMap.INTAKE_WRIST, MotorType.kBrushless);
    wristRotationEncoder = new DutyCycleEncoder(IntakeMap.SensorConstants.INTAKE_ROTATE_ENCODER);   
  }



  @Override
  public void periodic() {
    wristRotationEncoder.setPositionOffset(0);
    rawEncoderValue = wristRotationEncoder.get() ;
    rotationAngleValue = rawEncoderValue * 360;

     if(!wristRotationEncoder.isConnected()) {
      wristRotation.setVoltage(0);
    }
    // This method will be called once per scheduler run
    readWristAngle();
    readIntakeEncoder();
  }

  public void rotateWristPID() {
    double output = pid.calculate(getWristAngle());
    setWristVoltage(output);
  }

  public void setPIDTarget(double target) {
    setTarget(target);
    pid.setSetpoint(target);
  }

  public boolean pidAtSetpoint() {
    return pid.atSetpoint();
  }

  // GETTERS
  public double getWristVoltage() {
    return wristVoltage.get();
  }

  public double getWristAngle() {
    return rotationAngle.get();
  }

  public double getEncoderValue() {
    return rotationEncoderValue.get();
  }

  public double getTarget() {
    return rotationTarget.get();
  }

  public boolean getFailure() {
    return failure.get();
  }


  // SETTERS
  public void setWristVoltage(double voltage) {
    wristVoltage.set(voltage);
    // wristVoltageLog.append(voltage);

    if (voltage > 0) {
      setTowardIntake(false);
    } else {
      setTowardIntake(true);
    }

    wristRotation.set(voltage); 
  }

  public void readWristAngle() {
    rotationAngle.set(rotationAngleValue);
  }

  public void readIntakeEncoder() {
    double rotationValue = wristRotationEncoder.get();
    if (rotationValue > 300 / 360) {
      rotationValue -= 1;
    } else if (rotationValue < -50 / 360) {
      rotationValue += 1;
    }
    rotationEncoderValue.set(rotationValue);
  }

  public void setTarget(double target) {
    rotationTarget.set(target);
  }
  public void setTargetAMP(double target) {
    rotationTarget.set(target);
  }

  public void setFailure(boolean failureValue) {
    failure.set(failureValue);
  }

  public void setTowardIntake(boolean state) {
    towardShooter.set(state);

  }



  @Override
  public void simulationPeriodic() {
  }


  @Override
  public void setDefaultCommand() {
  }


  @Override
  public boolean isHealthy() {
    double motor_temp = wristRotation.getMotorTemperature();
    double motor_current = wristRotation.getOutputCurrent();
    double motor_voltage = wristRotation.getBusVoltage();
    if (motor_temp > 50) DriverStation.reportError("Wrist motor temperature is too high", false);
    if (motor_temp > 60 || motor_current > 40 || motor_voltage < 10) {
      return false;
    }
    return true;    
  }


  @Override
  public void Failsafe() {
    wristRotation.set(0);
  }
}




