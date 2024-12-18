// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;

public class IntakeWheels extends SubsystemABS {
  private CANSparkMax intakeMotor;
  private DoubleEntry intakeSpeed;
  private ShuffleboardTab tab;

public IntakeWheels(Subsystems part, String tabName) {
  super(part, tabName);
  tab = getTab();
  
  tab.addNumber("intake bus voltage",() -> intakeMotor.getBusVoltage());
  tab.addNumber("intake motor temperature",() -> intakeMotor.getMotorTemperature());
}

  @Override
  public void init() {
  intakeMotor = new CANSparkMax(IntakeMap.INTAKE_MOTOR, MotorType.kBrushless);
  intakeSpeed = ntTable.getDoubleTopic("wheels_voltage").getEntry(0);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public void setDefaultCommand() {}

  @Override
  public boolean isHealthy() {
    // SafetyManager.java has ! why? must fix but this should work in current state
    double motorTemp = intakeMotor.getMotorTemperature();
    @SuppressWarnings("unused")
    double motorVoltage = intakeMotor.getBusVoltage();
    if (motorTemp > 50) DriverStation.reportWarning("Intake motor temperature is too high", true);
    if (motorTemp > 80) return false;
    
    return true;
  }

  @Override
  public void Failsafe() {
    intakeSpeed.set(0);
    intakeMotor.set(0);
  }

  public void setMotorSpeed(double speed){
    intakeMotor.set(speed);
    intakeSpeed.set(speed);
  }

  
  
}
