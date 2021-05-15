package frc.robot.subsystems;

import static frc.robot.Constants.kTalonConfigTimeout;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import frc.robot.Constants.DriveConstants;
import java.util.Set;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DriveSubsystem extends MeasurableSubsystem {

  private static final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
  private final SwerveDrive swerveDrive;

  /**
   * Uses the Third Coast SwerveDrive.
   */
  public DriveSubsystem(TelemetryService telemetryService, ConsoleSubsystem consoleSubsystem) {
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

      var driveTalon = new TalonFX(i + 10);
      driveTalon.configFactoryDefault(kTalonConfigTimeout);
      driveTalon.configAllSettings(DriveConstants.getDriveTalonConfig(), kTalonConfigTimeout);
      driveTalon.enableVoltageCompensation(true);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      swerveModules[i] = moduleBuilder
          .azimuthTalon(azimuthTalon)
          .driveTalon(driveTalon)
          .wheelLocationMeters(wheelLocations[i])
          .build();

      swerveModules[i].loadAndSetAzimuthZeroReference();
      telemetryService.register(azimuthTalon);
      telemetryService.register(driveTalon);

      var azimuthName = String.format("azimuth %d", i);
      consoleSubsystem.register(new Measure(azimuthName,
          () -> azimuthTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF));
      consoleSubsystem.register(
          new Measure(azimuthName, () -> ((int) azimuthTalon.getSelectedSensorPosition()) & 0xFFF));
    }

    swerveDrive = new SwerveDrive(swerveModules);
    swerveDrive.resetGyro();
  }

  /**
   * Returns the swerve drive kinematics object for use during trajectory configuration.
   *
   * @return the configured kinemetics object
   */
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return swerveDrive.getKinematics();
  }

  /**
   * Resets the robot's position on the field.
   *
   * @param pose the current pose
   */
  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    logger.info("reset odometry with pose = {}", pose);
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return the pose of the robot (x and y ane in meters)
   */
  public Pose2d getPoseMeters() {
    return swerveDrive.getPoseMeters();
  }

  /**
   * Perform periodic swerve drive odometry update.
   */
  @Override
  public void periodic() {
    swerveDrive.periodic();
  }

  /**
   * Drive the robot with given x, y, and rotational velocities with open-loop velocity control.
   */
  public void drive(double vxMetersPerSecond, double vyMetersPerSecond,
      double omegaRadiansPerSecond) {
    swerveDrive.drive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, true);
  }

  /**
   * Move the robot with given x, y, and rotational velocities with closed-loop velocity control.
   */
  public void move(double vxMetersPerSecond, double vyMetersPerSecond,
      double omegaRadiansPerSecond, boolean isFieldOriented) {
    swerveDrive.move(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, isFieldOriented);
  }

  public void resetGyro() {
    swerveDrive.resetGyro();
  }

  public Rotation2d getHeading() {
    return swerveDrive.getHeading();
  }

  // Measurable Support

  @NotNull
  @Override
  public Set<Measure> getMeasures() {
    return Set
        .of(new Measure("Gyro Angle (degrees)", () -> swerveDrive.getHeading().getDegrees()));
  }


}
