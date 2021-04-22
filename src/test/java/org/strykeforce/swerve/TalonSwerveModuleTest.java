package org.strykeforce.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.mockito.ArgumentCaptor;

import static frc.robot.Constants.Drive.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

class TalonSwerveModuleTest {

  @Nested
  @DisplayName("Should get module state")
  class ShouldGetModuleState {

    private TalonSRX azimuthTalon;
    private TalonFX driveTalon;
    private TalonSwerveModule module;

    @BeforeEach
    void setUp() {
      azimuthTalon = mock(TalonSRX.class);
      driveTalon = mock(TalonFX.class);
      module =
          new TalonSwerveModule.Builder(azimuthTalon, driveTalon)
              .driveGearRatio(kDriveGearRatio)
              .wheelDiameterInches(kWheelDiameterInches)
              .driveMaximumMetersPerSecond(kDriveMaximumMetersPerSecond)
              .build();
    }

    @Test
    @DisplayName("drive speed with default encoder counts")
    void speedWithDefaultEncoderCounts() {
      when(driveTalon.getSelectedSensorVelocity()).thenReturn(20480.0);
      SwerveModuleState state = module.getState();
      assertEquals(3.657337448, state.speedMetersPerSecond, 1e-9);
    }

    @Test
    @DisplayName("drive speed with encoder counts")
    void speedWithEncoderCounts() {
      when(driveTalon.getSelectedSensorVelocity()).thenReturn(20480.0);
      module =
          new TalonSwerveModule.Builder(azimuthTalon, driveTalon)
              .driveGearRatio(kDriveGearRatio)
              .wheelDiameterInches(kWheelDiameterInches)
              .driveMaximumMetersPerSecond(kDriveMaximumMetersPerSecond)
              .azimuthEncoderCountsPerRevolution(4096)
              .driveEncoderCountsPerRevolution(2048)
              .build();
      SwerveModuleState state = module.getState();
      assertEquals(3.657337448, state.speedMetersPerSecond, 1e-9);
    }

    @ParameterizedTest
    @CsvSource({
      // azimuth encoder counts, azimuth angle
      // cardinals
      "0, 0",
      "1024, 90",
      "2048, 180",
      "3072, -90",
      // cardinals plus 4096
      "4096, 0",
      "5120, 90",
      "6144, 180",
      "7168, -90",
      // negatives
      "-0, 0",
      "-1024, -90",
      "-2048, -180",
      "-3072, 90",
      // negatives minus 4096
      "-4096, 0",
      "-5120, -90",
      "-6144, -180",
      "-7168, 90",
      // misc
      "11.377778, 1", // 4096/360
      "-11.377778, -1"
    })
    @DisplayName("azimuth angle with default encoder counts")
    void angleWithDefaultEncoderCounts(double counts, double expectedAngleDeg) {
      when(azimuthTalon.getSelectedSensorPosition()).thenReturn(counts);
      SwerveModuleState state = module.getState();
      var expectedRotation = Rotation2d.fromDegrees(expectedAngleDeg);
      assertEquals(expectedRotation, state.angle);
    }
  }

  @Nested
  @DisplayName("Should set module state")
  class ShouldSetModuleState {

    final ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
    private TalonSRX azimuthTalon;
    private TalonFX driveTalon;
    private TalonSwerveModule module;

    @BeforeEach
    void setUp() {
      azimuthTalon = mock(TalonSRX.class);
      driveTalon = mock(TalonFX.class);
      module =
          new TalonSwerveModule.Builder(azimuthTalon, driveTalon)
              .driveGearRatio(kDriveGearRatio)
              .wheelDiameterInches(kWheelDiameterInches)
              .driveMaximumMetersPerSecond(kDriveMaximumMetersPerSecond)
              .build();
    }

    @ParameterizedTest
    @CsvSource({
      // drive m/s, percent output
      "0, 0",
      "0.304778121, 0.079365079",
      "3.657337448, 0.9523809525",
    })
    @DisplayName("drive speed when open loop")
    void driveOpenLoopSpeed(double driveMetersPerSecond, double expectedPercentOutput) {
      when(azimuthTalon.getSelectedSensorPosition()).thenReturn(0.0);
      var desiredState = new SwerveModuleState(driveMetersPerSecond, new Rotation2d());
      module.setOpenLoopDesiredState(desiredState);
      verify(driveTalon).set(eq(ControlMode.PercentOutput), captor.capture());
      assertEquals(expectedPercentOutput, captor.getValue(), 1e-9);
    }

    @ParameterizedTest
    @CsvSource({
      // drive m/s, encoder counts per 100ms
      "0, 0",
      "0.304778121, 1707",
      "0.609556241, 3413",
      "1.219112483, 6827",
      "3.84020432, 21504"
    })
    @DisplayName("drive speed when closed loop")
    void driveClosedLoopSpeed(double driveMetersPerSecond, double expectedCountsPer100ms) {
      when(azimuthTalon.getSelectedSensorPosition()).thenReturn(0.0);
      var desiredState = new SwerveModuleState(driveMetersPerSecond, new Rotation2d());
      module.setClosedLoopDesiredState(desiredState);
      verify(driveTalon).set(eq(ControlMode.Velocity), captor.capture());
      assertEquals(expectedCountsPer100ms, captor.getValue(), 0.5);
    }
  }

  @Test
  @DisplayName("Rotation2d should work as expected")
  void rotation2DShouldWorkAsExpected() {
    assertEquals(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(-180));

    var a = Rotation2d.fromDegrees(175);
    var b = Rotation2d.fromDegrees(10);

    assertEquals(Rotation2d.fromDegrees(165), a.minus(b));
    assertEquals(Rotation2d.fromDegrees(185), a.plus(b));
    assertEquals(Rotation2d.fromDegrees(-175), a.plus(b));
    assertEquals(-175.0, a.plus(b).getDegrees());

    var c = new Rotation2d(Math.PI);
    assertEquals(Rotation2d.fromDegrees(-175), a.rotateBy(b));
    assertEquals(Rotation2d.fromDegrees(-5), a.rotateBy(c));

    assertEquals(Rotation2d.fromDegrees(-175), a.unaryMinus());

    var d = Rotation2d.fromDegrees(360);
    assertEquals(Rotation2d.fromDegrees(1), d.plus(Rotation2d.fromDegrees(1)));
  }
}
