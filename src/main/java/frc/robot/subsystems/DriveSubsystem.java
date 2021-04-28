package frc.robot.subsystems;

import static frc.robot.Constants.kTalonConfigTimeout;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.TalonSwerveModule;

public class DriveSubsystem extends SubsystemBase {

  private static final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);

  private final SwerveDrive swerveDrive;

  private final double[] angles = new double[]{0.0, 45.0, -45.0};
  private int angleIndex = 0;

  /**
   * Uses the Third Coast SwerveDrive.
   */
  public DriveSubsystem() {
    var moduleBuilder = new TalonSwerveModule.Builder()
        .driveGearRatio(DriveConstants.kDriveGearRatio)
        .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
        .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond);

    TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];
    Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();

    for (int i = 0; i < 4; i++) {
      var azimuthTalon = new TalonSRX(i);
      azimuthTalon.configFactoryDefault(kTalonConfigTimeout);
      azimuthTalon.configAllSettings(DriveConstants.getAzimuthTalonConfig(), kTalonConfigTimeout);
      azimuthTalon.enableCurrentLimit(true);
      azimuthTalon.enableVoltageCompensation(true);
      azimuthTalon.setNeutralMode(NeutralMode.Coast);

      var driveTalon = new TalonSRX(i + 10);
      driveTalon.configFactoryDefault(kTalonConfigTimeout);
      driveTalon.configAllSettings(DriveConstants.getDriveTalonConfig(), kTalonConfigTimeout);
      driveTalon.enableCurrentLimit(true);
      driveTalon.enableVoltageCompensation(true);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      swerveModules[i] = moduleBuilder
          .azimuthTalon(azimuthTalon)
          .driveTalon(driveTalon)
          .wheelLocationMeters(wheelLocations[i])
          .build();
    }

    swerveDrive = new SwerveDrive(swerveModules);
  }

  /**
   * Perform periodic swerve drive odometry update.
   */
  @Override
  public void periodic() {
    swerveDrive.periodic();
  }

  /**
   * Drive the robot with given x, y, and rotational velocities.
   */
  public void drive(double vxMetersPerSecond, double vyMetersPerSecond,
      double omegaRadiansPerSecond) {
    swerveDrive.drive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, false);
  }

  /**
   * Swerve drive debugging support.
   */
  public void next() {
    double angle = angles[angleIndex++ % angles.length];
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      desiredStates[i] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(angle));
    }
    swerveDrive.setModuleStates(desiredStates);
    logger.debug("Set azimuth to next angle: {}", angle);
  }
}
