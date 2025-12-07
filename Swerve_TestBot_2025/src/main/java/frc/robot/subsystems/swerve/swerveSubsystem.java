package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemManager;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveMath;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class swerveSubsystem extends SubsystemBase {
    private static SwerveDrive swerveDrive;

    private final SwerveDrive swerveDrive;
    private final ADIS16470_IMU gyro;

    int frontLeftDriveID  = 1;
    int frontLeftSteerID  = 2;
    double frontLeftOffset = 0.0;

    int frontRightDriveID = 3;
    int frontRightSteerID = 4;
    double frontRightOffset = 0.0;

    int backLeftDriveID   = 5;
    int backLeftSteerID   = 6;
    double backLeftOffset = 0.0;

    int backRightDriveID  = 7;
    int backRightSteerID  = 8;
    double backRightOffset = 0.0;

    public SwerveSubsystem() {

        // change if needed
        gyro = new ADIS16470_IMU();
        gyro.reset();

        swerveDrive = new SwerveDrive(frontLeftDriveID, frontLeftSteerID, frontLeftOffset,
        frontRightDriveID, frontRightSteerID, frontRightOffset,
        backLeftDriveID, backLeftSteerID, backLeftOffset,
        backRightDriveID, backRightSteerID, backRightOffset,
        gyro);        
    }


    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {

            double x = translationX.getAsDouble();
            double y = translationY.getAsDouble();
            double rot = angularRotationX.getAsDouble();

            //deadband
            x = MathUtil.applyDeadband(x, 0.05);
            y = MathUtil.applyDeadband(y, 0.05);
            rot = MathUtil.applyDeadband(rot, 0.05);

            Translation2d translation = SwerveMath.scaleTranslation(new Translation2d(
                x * swerveDrive.getMaximumChassisVelocity(),
                y * swerveDrive.getMaximumChassisVelocity()), 0.8);
            double omegaRadiansPerSecond = Math.pow(rot, 3) * swerveDrive.getMaximumChassisVelocity();


            swerveDrive.drive(translation, 
                            omegaRadiansPerSecond,
                            true,
                            false);
        });
    }
    

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, 
                        rotation, 
                        fieldRelative, 
                        false);
    }

}