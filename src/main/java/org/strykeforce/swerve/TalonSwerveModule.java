package org.strykeforce.swerve;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;

import java.util.Objects;

public class TalonSwerveModule implements SwerveModule {
  final int k100msPerSecond = 10;

  private final BaseTalon azimuthTalon;
  private final BaseTalon driveTalon;
  private final double azimuthCountsPerRev;
  private final double driveCountsPerRev;
  private final double driveGearRatio;
  private final double wheelCircumferenceMeters;

  private TalonSwerveModule(Builder builder) {
    this.azimuthTalon = builder.azimuthTalon;
    this.driveTalon = builder.driveTalon;
    this.azimuthCountsPerRev = builder.azimuthCountsPerRev;
    this.driveCountsPerRev = builder.driveCountsPerRev;
    driveGearRatio = builder.driveGearRatio;
    this.wheelCircumferenceMeters = Math.PI * Units.inchesToMeters(builder.wheelDiameterInches);
  }

  @Override
  public SwerveModuleState getState() {
    double azimuthPosition = azimuthTalon.getSelectedSensorPosition();
    var angle = new Rotation2d(azimuthCountsToRadians(azimuthPosition));

    double driveVelocity = driveTalon.getSelectedSensorVelocity();
    double speedMetersPerSecond = driveCountsToMetersPerSecond(driveVelocity);

    return new SwerveModuleState(speedMetersPerSecond, angle);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {}

  private double azimuthCountsToRadians(double encoderCounts) {
    double azimuthCounts = Math.IEEEremainder(encoderCounts, azimuthCountsPerRev);
    return (azimuthCounts / azimuthCountsPerRev) * (2.0 * Math.PI);
  }

  private double driveCountsToMetersPerSecond(double encoderCountsPer100ms) {
    double motorRotationsPer100ms = encoderCountsPer100ms / driveCountsPerRev;
    double wheelRotationsPer100ms = motorRotationsPer100ms * driveGearRatio;
    double metersPer100ms = wheelRotationsPer100ms * wheelCircumferenceMeters;
    return metersPer100ms * k100msPerSecond;
  }

  public static class Builder {
    public static final int kDefaultTalonSRXCountsPerRev = 4096;
    public static final int kDefaultTalonFXCountsPerRev = 2048;

    private final BaseTalon azimuthTalon;
    private final BaseTalon driveTalon;

    private double driveGearRatio;
    private double wheelDiameterInches;
    private int driveCountsPerRev = kDefaultTalonFXCountsPerRev;
    private int azimuthCountsPerRev = kDefaultTalonSRXCountsPerRev;

    public Builder(BaseTalon azimuthTalon, BaseTalon driveTalon) {
      this.azimuthTalon = Objects.requireNonNull(azimuthTalon);
      this.driveTalon = Objects.requireNonNull(driveTalon);
    }

    public Builder driveGearRatio(double ratio) {
      driveGearRatio = ratio;
      return this;
    }

    public Builder wheelDiameterInches(double diameterInches) {
      wheelDiameterInches = diameterInches;
      return this;
    }

    public Builder driveEncoderCountsPerRevolution(int countsPerRev) {
      driveCountsPerRev = countsPerRev;
      return this;
    }

    public Builder azimuthEncoderCountsPerRevolution(int countsPerRev) {
      azimuthCountsPerRev = countsPerRev;
      return this;
    }

    public TalonSwerveModule build() {
      return new TalonSwerveModule(this);
    }
  }
}
