package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class Tester {
    private MotorClass motorClass = new MotorClass();

    @Test
    public void Motors_Test() {
        // Use reflection to get all motor fields
        Field[] fields = MotorClass.class.getDeclaredFields();
        if ( !Utils.isSimulation() ) {
            for ( Field field : fields ) {
                try {
                    // Get the CAN ID of the motor
                    int canId = field.getInt(motorClass);
                    try (// Create a TalonFX motor instance
                    TalonFX motor = new TalonFX(canId)) {
                        // Check if the motor is alive
                        boolean isAlive = motor.isAlive();

                        // Assert that the motor is alive
                        assertTrue(isAlive , "Motor with CAN ID " + canId + " is not alive.");
                    }
                } catch ( IllegalAccessException e ) {
                    e.printStackTrace();
                }
            }
        }
        else {

            System.out.println("\033[1;34m\033[1mTest skipped because we are in simulation mode.\033[0m");
        }
    }
}