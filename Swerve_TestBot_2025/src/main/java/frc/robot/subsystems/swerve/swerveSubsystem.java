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
import edu.wpi.first.math.util.MathUtil;
import com.ctre.phoenix6.hardware.Pigeon2;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;
    private final Pigeon2 gyro;

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

        gyro = new Pigeon2(0);
        gyro.setYaw(0);

        swerveDrive = new SwerveDrive(frontLeftDriveID, frontLeftSteerID, frontLeftOffset,
        frontRightDriveID, frontRightSteerID, frontRightOffset,
        backLeftDriveID, backLeftSteerID, backLeftOffset,
        backRightDriveID, backRightSteerID, backRightOffset,
        gyro);        
    }


    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {

            double vx = translationX.getAsDouble();
            double vy = translationY.getAsDouble();
            double omega = angularRotationX.getAsDouble();

            //deadband
            vx = MathUtil.applyDeadband(vx, 0.05);
            vy = MathUtil.applyDeadband(vy, 0.05);
            omega = MathUtil.applyDeadband(omega, 0.05);

            Translation2d translation = SwerveMath.scaleTranslation(new Translation2d(
                vx * swerveDrive.getMaximumChassisVelocity(),
                vy * swerveDrive.getMaximumChassisVelocity()), 0.8);
            double omegaRadiansPerSecond = Math.pow(omega, 3) * swerveDrive.getMaximumChassisAngularVelocity();


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