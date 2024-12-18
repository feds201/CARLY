package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;

public class Elevator extends SubsystemABS {
    private TalonFX elevatorMotor;
    private BooleanSupplier enabled;
    private DoubleSupplier voltage;
    private int counter = 0;
    private int seconds = 0;
    private boolean isEnabled = false;
    private double voltageValue = 0;

    public Elevator(Subsystems part, String tabname) {
        super(part, tabname);
        tab = getTab();
        tab.addBoolean("Toggle Elevator", enabled);
        tab.addNumber("Voltage", voltage);
        tab.addNumber("Seconds", () -> seconds);
        tab.addNumber("Counter", () -> counter);
        
        tab.addNumber("Output Current", () -> elevatorMotor.getSupplyCurrent().getValue());
        tab.addNumber("Output Voltage", () -> elevatorMotor.getMotorVoltage().getValue());
    }

    @Override
    public void init() {
        elevatorMotor = new TalonFX(ElevatorMap.ELEVATOR_MOTOR);
        enabled = () -> false;
        voltage = () -> 0;
        resetElevator();
    }

    public void setElevator(double power){
        elevatorMotor.set(power);
    }

    private void resetElevator() {
        isEnabled = false;
        voltageValue = 0;
        seconds = 0;
        counter = 0;
    }

    @Override
    public void periodic() {
        updateElevatorState();
    }

    @Override
    public void simulationPeriodic() {
        updateElevatorState();
        if (isEnabled) {
            simulateElevatorOperation();
        }
    }

    private void updateElevatorState() {
        isEnabled = enabled.getAsBoolean();
        voltageValue = voltage.getAsDouble();
    }

    private void simulateElevatorOperation() {
        if (counter % 30 == 0) {
            seconds++;
        }
        if (isEnabled && counter == 0) {
            voltageValue = 10;
        }
        counter++;
        if (seconds >= 20) {
            stopElevator();
        }

    
        
    }

    private void stopElevator() {
        elevatorMotor.stopMotor();
        voltageValue = 0;
        resetElevator();
    }

    @Override
    public void setDefaultCommand() {
        // No default command set
    }

    @Override
    public boolean isHealthy() {
        return true;
    }

    @Override
    public void Failsafe() {
        stopElevator();
    }


}
