package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;



public class SwerveDrive extends SubsystemBase {

    // Attributes
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    Pigeon2 gyro;
    SwerveModule[]        swerveModules; // Psuedo-class representing swerve modules.

    

    double Xpos = translationX.getAsDouble();
    double Ypos = translationY.getAsDouble();
    double omega = angularRotationX.getAsDouble();
    double omegaRadiansPerSecond = Math.pow(omega, 3) * swerveDrive.getMaximumChassisAngularVelocity();


    
    // Constructor
    public SwerveDrive() {  
    
        swerveModules = new SwerveModule[4]; // Psuedo-code; Create swerve modules here.
        
        // Create SwerveDriveKinematics object
        // 12.5in from center of robot to center of wheel.
        // 12.5in is converted to meters to work with object.
        // Translation2d(x,y) == Translation2d(front, left)
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))  // Back Right
        );
        
        gyro = new Pigeon2(0);

        // Create the SwerveDriveOdometry given the current angle, the robot is at x=0, r=0, and heading=0
        odometry = new SwerveDriveOdometry(
            kinematics,
            heading, // returns current gyro reading as a Rotation2d
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
            // Front-Left, Front-Right, Back-Left, Back-Right
            new Pose2d(0,0,new Rotation2d())
        );
            
    }
    
    // Simple drive function
    public void drive() {
        // Create test ChassisSpeeds
        ChassisSpeeds testSpeeds = new ChassisSpeeds(Xpos, Ypos, omegaRadiansPerSecond);
        // Get the SwerveModuleStates for each module given the desired speeds.
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);
        // Output order is Front-Left, Front-Right, Back-Left, Back-Right
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, swerveDrive.getMaximumSpeedMetersPerSecond());

        swerveModules[0].setState(swerveModuleStates[0]);
        swerveModules[1].setState(swerveModuleStates[1]);
        swerveModules[2].setState(swerveModuleStates[2]);
        swerveModules[3].setState(swerveModuleStates[3]);
    }
    
    // Fetch the current swerve module positions.
    public SwerveModulePosition[] getCurrentSwerveModulePositions() {
        return new SwerveModulePosition[]{
            new SwerveModulePosition(swerveModules[0].getDistance(), swerveModules[0].getAngle()), // Front-Left
            new SwerveModulePosition(swerveModules[1].getDistance(), swerveModules[1].getAngle()), // Front-Right
            new SwerveModulePosition(swerveModules[2].getDistance(), swerveModules[2].getAngle()), // Back-Left
            new SwerveModulePosition(swerveModules[3].getDistance(), swerveModules[3].getAngle())  // Back-Right
        };
    }
    
    @Override
    public void periodic() {

        SwerveSubsystem.periodic();
    }
}