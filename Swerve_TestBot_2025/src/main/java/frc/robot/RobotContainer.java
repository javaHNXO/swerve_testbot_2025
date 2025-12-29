package frc.robot.subsystems.swerve;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SubsystemManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  final CommandPS4Controller driver = new commandPS4Controller(0);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getdeployDirectory(), "swerve/falcon"));

  SwerveInputStream driveAngularVelocity = new SwerveInputStream.of(drivebase.getSwerveDrive(), 
                                                                    () -> -driver.getLeftY(),
                                                                    () -> -driver.getleftX())
                                                                    .withControllerRotationAxis(driver::getRightX())
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()withControllerHeadingAxis(driver::getRightX,
                                                                                            driver::getRightY)
                                                                                            .headingWhile(true);
  
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> -driver.getLeftY(),
                                                                    () -> -driver.getleftX())
                                                                    .withControllerRotationAxis(() -> driver.getRawAxis(2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  
  SwerveInputStream driveDirectionAngleKeyboard = driveAngularVelocityKeyboard.copy()withControllerHeadingAxis(
                                                                            () -> math.sin(driver.getRawAxis(2) * Math.PI)
                                                                            * (Math.PI * 2),
                                                                            () -> math.cos(driver.getRawAxis(2) * Math.PI)
                                                                            * (Math.PI * 2))
                                                  .headingWhile(true).translationHeadingOffset(true)
                                                  .translationHeadingOffsetAngle(Rotaion2d.fromDegrees(0));

  


  public static void init(){
    
    SubsystemManager.init();
    configureBindings();

  }
  // The robot's subsystems and commands are defined here...

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public static void configureBindings() {
    
    SubsystemManager.swerve.setDefaultCommand( 

      SwerveSubsystem.driveCommand(
          () -> driver.getLeftX(),
          () -> -driver.getLeftY(),
          () -> driver.getRightX()
      )
    );
  }
}
