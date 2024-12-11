// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;

public class IntakeIRSensor extends SubsystemABS {
  /** Creates a new BreakBeamSensor. */
  // private final DigitalInput transmitter;
  private final DigitalInput receiverIntake;
  private final NetworkTableEntry beamBrokenIntake;
  private final ShuffleboardTab tab;
  private final NetworkTableEntry nTableEntry;
 
  public IntakeIRSensor(Subsystems part, String tabName) {
    super(part, tabName);  
    tab = getTab();
    receiverIntake = new DigitalInput(0);
    setupNetworkTables(tabName);
    nTableEntry = ntTable.getEntry("Intake IR Sensor"); 
    beamBrokenIntake = ntTable.getEntry(tabName);
    
  }

  public boolean getBeamBroken() {
    return receiverIntake.get();
  }
  

  @Override
  public void init() {
    
  }

  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub
   
  }

  @Override
  public void setDefaultCommand() {
    // TODO Auto-generated method stub
    
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
