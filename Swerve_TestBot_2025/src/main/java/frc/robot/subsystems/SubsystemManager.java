package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class SubsystemManager {
    public static void init() {
        
    }

    private static final CommandPS4Controller ps4Joystick = new CommandPS4Controller(0);
    private static final PS4Controller psControllerHID = ps4Joystick.getHID();

    public static CommandPS4Controller getPSJoystick() {
        return ps4Joystick;
    }
    

}