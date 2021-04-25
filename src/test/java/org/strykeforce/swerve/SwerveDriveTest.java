package org.strykeforce.swerve;

import static frc.robot.Constants.Drive.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.Drive.kWheelLocations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.mockito.ArgumentCaptor;

class SwerveDriveTest {

  final ArgumentCaptor<SwerveModuleState> captor = ArgumentCaptor.forClass(SwerveModuleState.class);

  @Test
  @DisplayName("Should throw on multiple max speeds")
  void shouldThrowOnMultipleMaxSpeeds() {
    SwerveModule[] swerveModules = new SwerveModule[4];

    for (int i = 0; i < 4; i++) {
      swerveModules[i] = mock(SwerveModule.class);
      when(swerveModules[i].getWheelLocationMeters()).thenReturn(kWheelLocations[i]);
      when(swerveModules[i].getMaxSpeedMetersPerSecond()).thenReturn(1.0 * i);
    }

    assertThrows(IllegalStateException.class, () -> {
      SwerveDrive swerveDrive = new SwerveDrive(swerveModules);
    });
  }

  @ParameterizedTest
  @CsvFileSource(resources = "/swerve_test_cases.csv", numLinesToSkip = 1)
  @DisplayName("Should produce correct swerve module states")
  void shouldProduceCorrectSwerveModuleStates(double vxMetersPerSecond, double vyMetersPerSecond,
      double omegaRadiansPerSecond, boolean isFieldOriented, double gyroAngle, double lfAngle, double lfSpeed,
      double rfAngle, double rfSpeed, double lrAngle, double lrSpeed, double rrAngle,
      double rrSpeed) {

    SwerveModule[] swerveModules = new SwerveModule[4];

    for (int i = 0; i < 4; i++) {
      swerveModules[i] = mock(SwerveModule.class);
      when(swerveModules[i].getWheelLocationMeters()).thenReturn(kWheelLocations[i]);
      when(swerveModules[i].getMaxSpeedMetersPerSecond()).thenReturn(kMaxSpeedMetersPerSecond);
    }

    Gyro gyro = mock(Gyro.class);
    when(gyro.getRotation2d()).thenReturn(Rotation2d.fromDegrees(gyroAngle));
    SwerveDrive swerveDrive = new SwerveDrive(gyro, swerveModules);

    swerveDrive.drive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, isFieldOriented);
    verify(swerveModules[0]).setDesiredState(captor.capture(), eq(true));
    SwerveModuleState state = captor.getValue();

    assertEquals(lfAngle, state.angle.getDegrees(), 1e-9, "left front angle");
    assertEquals(lfSpeed, state.speedMetersPerSecond, 1e-9, "left front speed");

    verify(swerveModules[1]).setDesiredState(captor.capture(), eq(true));
    state = captor.getValue();

    assertEquals(rfAngle, state.angle.getDegrees(), 1e-9, "right front angle");
    assertEquals(rfSpeed, state.speedMetersPerSecond, 1e-9, "right front speed");

    verify(swerveModules[2]).setDesiredState(captor.capture(), eq(true));
    state = captor.getValue();

    assertEquals(lrAngle, state.angle.getDegrees(), 1e-9, "left rear angle");
    assertEquals(lrSpeed, state.speedMetersPerSecond, 1e-9, "left rear speed");

    verify(swerveModules[3]).setDesiredState(captor.capture(), eq(true));
    state = captor.getValue();

    assertEquals(rrAngle, state.angle.getDegrees(), 1e-9, "right rear angle");
    assertEquals(rrSpeed, state.speedMetersPerSecond, 1e-9, "right rear speed");
  }

}