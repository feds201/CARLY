// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotMap;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;

public class IntakeWheels extends SubsystemABS {
  private CANSparkMax intakeMotor;
  private DoubleEntry intakeSpeed;
  private ShuffleboardTab tab;

public IntakeWheels(Subsystems part, String tabName) {
  super(part, tabName);
  tab = getTab();
  intakeMotor = new CANSparkMax(RobotMap.IntakeMap.INTAKE_MOTOR, MotorType.kBrushless);
  intakeSpeed = ntTable.getDoubleTopic("wheels_voltage").getEntry(0);
}

  @Override
  public void init() {}

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    SmartDashboard.putNumber("intake bus voltage", intakeMotor.getBusVoltage());
    SmartDashboard.putNumber("intake motor temperature", intakeMotor.getMotorTemperature());
    throw new UnsupportedOperationException("Unimplemented method 'periodic'");

  }

  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'simulationPeriodic'");
  }

  @Override
  public void setDefaultCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDefaultCommand'");
  }

  @Override
  public boolean isHealthy() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'isHealthy'");
  }

  @Override
  public void Failsafe() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'Failsafe'");
  }

  public void setMotorSpeed(double speed){
    intakeMotor.set(speed);
    intakeSpeed.set(speed);
  }

  
  
}
