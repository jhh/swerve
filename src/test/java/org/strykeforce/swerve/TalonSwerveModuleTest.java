package org.strykeforce.swerve;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import static frc.robot.Constants.Drive.kDriveGearRatio;
import static frc.robot.Constants.Drive.kWheelDiameterInches;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.Mockito.*;

class TalonSwerveModuleTest {

  @Nested
  @DisplayName("Should get module state")
  class ShouldGetModuleState {

    @Test
    @DisplayName("speed with default encoder counts")
    void speedWithDefaultEncoderCounts() {
      TalonSRX azimuthTalon = mock(TalonSRX.class);
      TalonFX driveTalon = mock(TalonFX.class);

      when(driveTalon.getSelectedSensorVelocity()).thenReturn(20480.0);

      TalonSwerveModule module =
          new TalonSwerveModule.Builder(azimuthTalon, driveTalon)
              .driveGearRatio(kDriveGearRatio)
              .wheelDiameterInches(kWheelDiameterInches)
              .build();

      SwerveModuleState state = module.getState();

      assertEquals(3.657337448, state.speedMetersPerSecond, 1e-9);
    }

    @Test
    @DisplayName("speed with encoder counts")
    void speedWithEncoderCounts() {
      TalonSRX azimuthTalon = mock(TalonSRX.class);
      TalonFX driveTalon = mock(TalonFX.class);

      when(driveTalon.getSelectedSensorVelocity()).thenReturn(20480.0);

      TalonSwerveModule module =
          new TalonSwerveModule.Builder(azimuthTalon, driveTalon)
              .driveGearRatio(kDriveGearRatio)
              .wheelDiameterInches(kWheelDiameterInches)
              .azimuthEncoderCountsPerRevolution(4096)
              .driveEncoderCountsPerRevolution(2048)
              .build();

      SwerveModuleState state = module.getState();

      assertEquals(3.657337448, state.speedMetersPerSecond, 1e-9);
    }

    @ParameterizedTest
    @CsvSource({
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
    @DisplayName("angle with default encoder counts")
    void angleWithDefaultEncoderCounts(double counts, double expectedAngleDeg) {
      TalonSRX azimuthTalon = mock(TalonSRX.class);
      TalonFX driveTalon = mock(TalonFX.class);

      when(azimuthTalon.getSelectedSensorPosition()).thenReturn(counts);

      TalonSwerveModule module =
          new TalonSwerveModule.Builder(azimuthTalon, driveTalon)
              .driveGearRatio(kDriveGearRatio)
              .wheelDiameterInches(kWheelDiameterInches)
              .build();

      SwerveModuleState state = module.getState();
      var expectedRotation = Rotation2d.fromDegrees(expectedAngleDeg);
      assertEquals(expectedRotation, state.angle);
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

  }
}
