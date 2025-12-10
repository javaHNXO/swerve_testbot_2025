package frc.robot;


import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;

import java.io.File;
import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;
    private final Pigeon2 gyro;
    private SwerveDriveOdometry odometry;

    private final File swerveJsonDir = new File(Filesystem.getDeployDirectory(), "swerve");  

    public SwerveSubsystem() {

        gyro = new Pigeon2(0);
        gyro.setYaw(0); 
        odometry = new SwerveDriveOdometry(swerveDrive.getKinematics(), Rotation2d.fromDegrees(gyro.getYaw()));

        try {
        
            swerveDrive = new SwerveDrive(swerveConfigFolder);

        } catch (Exception e) {
            DriverStation.reportWarning("WARNING: Swerve JSON files not found yet.");
            DriverStation.reportWarning("Place them in: deploy/swerve/");
            DriverStation.reportWarning("Robot will not drive until JSON is added.");
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean IsOpenLoop) {

        if (swerveDrive == null) return;  // kogda budut json fayli, udalit nahui

        if (fieldRelative) {

            swerveDrive.driveFieldOriented(translation.getX(), translation.getY(), rotation, gyro.getRotation2d());

        } else {

            swerveDrive.drive(translation.getX(), translation.getY(), rotation);
        }
    }


    public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return run(() -> {

            double vx = x.getAsDouble();
            double vy = y.getAsDouble();
            double omega = rot.getAsDouble();

            //deadband
            vx = MathUtil.applyDeadband(x, 0.05);
            vy = MathUtil.applyDeadband(y, 0.05);
            omega = MathUtil.applyDeadband(rot, 0.05);

            Translation2d translation = SwerveMath.scaleTranslation(new Translation2d(
                vx * swerveDrive.getMaximumChassisVelocity(),
                vy * swerveDrive.getMaximumChassisVelocity()), 0.8);
            double omegaRadiansPerSecond = Math.pow(omega, 3) * swerveDrive.getMaximumChassisAngularVelocity();


            drive(translation, omegaRadiansPerSecond, true, false);
        });
    }

    @Override
    public void periodic() {

        Rotation2d heading = Rotation2d.fromDegrees(gyro.getYaw());

        odometry.update(heading,  getCurrentSwerveModulePositions());
    }

}