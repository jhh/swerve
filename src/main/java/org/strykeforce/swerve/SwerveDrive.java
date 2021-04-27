package org.strykeforce.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SwerveDrive {

  private static final Logger logger = LoggerFactory.getLogger(SwerveDrive.class);

  private final SwerveModule[] swerveModules;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private final Gyro gyro;
  private final double maxSpeedMetersPerSecond;

  public SwerveDrive(Gyro gyro, SwerveModule... swerveModules) {
    this.gyro = gyro;
    this.swerveModules = swerveModules;
    final List<Translation2d> locations = Arrays.stream(swerveModules)
        .map(SwerveModule::getWheelLocationMeters)
        .collect(Collectors.toList());

    Translation2d[] translation2ds = Arrays.stream(swerveModules)
        .map(SwerveModule::getWheelLocationMeters)
        .toArray(Translation2d[]::new);

    // verify all swerve modules are set to same max speed
    Set<Double> maxSpeeds = Arrays.stream(swerveModules)
        .map(SwerveModule::getMaxSpeedMetersPerSecond)
        .collect(Collectors.toSet());

    if (maxSpeeds.size() > 1) {
      throw new IllegalStateException("swerve modules must have same driveMaximumMetersPerSecond");
    }
    maxSpeedMetersPerSecond = swerveModules[0].getMaxSpeedMetersPerSecond();

    kinematics = new SwerveDriveKinematics(translation2ds);
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
  }

  public SwerveDrive(SwerveModule... swerveModules) {
    this(new AHRS(), swerveModules);
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  public Rotation2d getHeading() {
    return gyro.getRotation2d();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void resetDriveEncoders() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].resetDriveEncoder();
    }
  }

  public void periodic() {
    odometry.update(gyro.getRotation2d(), swerveModules[0].getState(), swerveModules[1].getState(),
        swerveModules[2].getState(), swerveModules[3].getState());
  }

  public void drive(double vxMetersPerSecond, double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      boolean isFieldOriented) {
    ChassisSpeeds chassisSpeeds = isFieldOriented ? ChassisSpeeds
        .fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond,
            gyro.getRotation2d())
        : new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

    var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, maxSpeedMetersPerSecond);

    for (int i = 0; i < 4; i++) {
      swerveModules[i].setDesiredState(swerveModuleStates[i], true);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, maxSpeedMetersPerSecond);
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setDesiredState(desiredStates[i], true);
    }
  }

}
