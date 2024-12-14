package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ComandCenter {
    public static ShuffleboardTab tab;
    public static void init(){
        tab = Shuffleboard.getTab("Comand Center");
    }

    public static void add(String name, Object value){
        tab.add(name, value);
    }
    public static void add(String name, Object value, String widget){
        tab.add(name, value).withWidget(widget);
    }
    
    public static void add(String name, Object value, String widget, String title){
        tab.add(name, value).withWidget(widget).withProperties(Map.of("Title", title));
    }
    public static void add(String name, Object value, String widget, String title, String tabName){
        tab.add(name, value).withWidget(widget).withProperties(Map.of("Title", title)).withProperties(Map.of("Tab", tabName));
    }


}
