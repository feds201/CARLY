// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.RobotMap.SensorMap;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;

public class IntakeIRSensor extends SubsystemABS {
  /** Creates a new BreakBeamSensor. */
  // private final DigitalInput transmitter;
  private  DigitalInput receiverIntake;
  public IntakeIRSensor(Subsystems part, String tabName) {
    super(part, tabName);  
    ntTable.getEntry("Intake IR Sensor"); 
    ntTable.getEntry(tabName);
    
  }

  public boolean getBeamBroken() {
    return !receiverIntake.get();
  }
  

  @Override
  public void init() {
     tab.addBoolean("NoteTaken ", () -> getBeamBroken());
    try {
      receiverIntake = new DigitalInput(SensorMap.INTAKE_IR_SENSOR);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void simulationPeriodic() {
   
  }

  @Override
  public void setDefaultCommand() {
    
  }

  @Override
  public boolean isHealthy() {
    return receiverIntake != null;
  }

  @Override
  public void Failsafe() {
    System.out.println("Intake IR Sensor is not healthy");
  }

  @Override
  public void periodic() {
  }

}
