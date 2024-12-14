// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
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
    return receiverIntake.get();
  }
  

  @Override
  public void init() {
    /*TODO: Verify the dio port for this sensor*/
    try {
      receiverIntake = new DigitalInput(5);
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
    return true;
  }

  @Override
  public void Failsafe() {
    // TODO Auto-generated method stub
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
  }

}
