package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;

public class Elevator extends SubsystemABS {

    private ShuffleboardTab tab;
    private TalonFX elevatorMotor;
    private BooleanSupplier enabled;
    private DoubleSupplier voltage;
    private int counter = 0;
    private int seconds = 0;
    private boolean isEnabled = false;
    private double voltageValue = 0;

    public Elevator(Subsystems subsystem, String tab_string) {
        super(subsystem, tab_string);
        tab = getTab();
    }

    @Override
    public void init() {
        elevatorMotor = new TalonFX(ElevatorMap.ELEVATOR_MOTOR);
        enabled = () -> false;
        voltage = () -> 0;
        tab.addBoolean("Toggle Elevator", enabled);
        tab.addNumber("Voltage", voltage);
        resetElevator();
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

        // Example output to console instead of SmartDashboard
        System.out.println("Output Current: " + voltageValue);
        System.out.println("Counter: " + counter);
        System.out.println("Seconds: " + seconds);
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
