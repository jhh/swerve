package frc.robot.subsystems;

import static frc.robot.Constants.kTalonConfigTimeout;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.TalonSwerveModule;

public class DriveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;

  private final double[] angles = new double[]{0.0, 45.0, -45.0};
  private int angleIndex = 0;

  public DriveSubsystem() {
    var builder = new TalonSwerveModule.Builder()
        .driveGearRatio(Drive.kDriveGearRatio)
        .wheelDiameterInches(Drive.kWheelDiameterInches)
        .driveMaximumMetersPerSecond(Drive.kMaxSpeedMetersPerSecond);

    TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];

    for (int i = 0; i < 4; i++) {
      var azimuthTalon = new TalonSRX(i);
      azimuthTalon.configFactoryDefault(kTalonConfigTimeout);
      azimuthTalon.configAllSettings(Drive.kAzimuthTalonConfiguration, kTalonConfigTimeout);
      azimuthTalon.enableCurrentLimit(true);
      azimuthTalon.enableVoltageCompensation(true);
      azimuthTalon.setNeutralMode(NeutralMode.Coast);

      var driveTalon = new TalonSRX(i + 10);
      driveTalon.configFactoryDefault(kTalonConfigTimeout);
      driveTalon.configAllSettings(Drive.kDriveTalonConfiguration, kTalonConfigTimeout);
      driveTalon.enableCurrentLimit(true);
      driveTalon.enableVoltageCompensation(true);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      swerveModules[i] = builder
          .azimuthTalon(azimuthTalon)
          .driveTalon(driveTalon)
          .wheelLocationMeters(Drive.kWheelLocations[i])
          .build();

    }

    swerveDrive = new SwerveDrive(swerveModules);
  }

  public void drive(double vxMetersPerSecond, double vyMetersPerSecond,
      double omegaRadiansPerSecond) {
    swerveDrive.drive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, false);
  }

  public void next() {
    double angle = angles[angleIndex++ % angles.length];
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      desiredStates[i] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(angle));
    }
    swerveDrive.setModuleStates(desiredStates);
  }
}
