package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SubsystemManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static CommandPS4Controller driver;

  public static void init(){
    driver = new CommandPS4Controller(0);
    SubsystemManager.init();
    configureBindings();

  }
  // The robot's subsystems and commands are defined here...

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public static void configureBindings() {
    swerve.setDefaultCommand(
      SwerveSubsystem.driveCommand(
          () -> driver.getLeftX(),
          () -> -driver.getLeftY(),
          () -> driver.getRightX()
      )
    );
  }
}
