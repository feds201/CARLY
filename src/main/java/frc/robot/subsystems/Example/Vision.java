package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.SubsystemABS;
import frc.robot.subsystems.Subsystems;

public class Vision extends SubsystemABS {
    ShuffleboardTab tab;
    Boolean isVisionActive;
    public Vision(Subsystems part , String tabName) {
        super(part , tabName);
    }

    @Override
    public void init() {
        isVisionActive = true;
        tab = getTab();
        // Initialize vision-related components here


    }

    @Override
    public void periodic() {


    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void setDefaultCommand() {

    }

    @Override
    public boolean isHealthy() {
        return isVisionActive;
    }
}
