package org.strykeforce.swerve;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static frc.robot.Constants.Drive.kDriveGearRatio;
import static frc.robot.Constants.Drive.kWheelDiameterInches;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;
import static org.mockito.Mockito.*;

class TalonSwerveModuleTest {

  @Test
  @DisplayName("Should calculate module state speed")
  void shouldGetModuleStateSpeed() {
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
  void smoke() {
    assertEquals(2, 1 + 1);

    TalonSRX driveTalon = mock(TalonSRX.class);
    driveTalon.configFactoryDefault(10);

    verify(driveTalon).configFactoryDefault(10);
  }
}
