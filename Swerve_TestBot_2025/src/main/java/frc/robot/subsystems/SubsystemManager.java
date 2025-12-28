package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class SubsystemManager {
    public static SwerveSubsystem swerve;

    public static void init() {
        swerve = new SwerveSubsystem();
    }
}