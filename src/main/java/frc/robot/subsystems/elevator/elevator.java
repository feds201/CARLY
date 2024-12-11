// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.RobotMap;
import frc.robot.constants.ElevatorConstants;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;

public class elevator extends SubsystemABS {
  /** Creates a new elevator. */

  public static TalonFX elevatorMotor = new TalonFX(RobotMap.ElevatorMap.ELEVATOR_MOTOR);

  DigitalInput toplimitSwitch = new DigitalInput(0);
  DigitalInput bottomlimitSwitch = new DigitalInput(1);

  private ShuffleboardTab tab;
  private String tabName;

  public elevator(Subsystems part , String tabName) {
    super(part, tabName);
    this.tabName = tabName;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    @Override
    public void init() {
        tab = getTab();
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void setDefaultCommand() {

    }

    @Override
    public boolean isHealthy() {
        return false;
    }

    public void moveUp(){
        elevatorMotor.set(ElevatorConstants.kElevatorSpeed);
    }

    public void moveDown(){
        elevatorMotor.set(-1 * ElevatorConstants.kElevatorSpeed);
    }

    @Override
    public void Failsafe() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'Failsafe'");
    }


    }
