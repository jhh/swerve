package org.strykeforce.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  private final double driveMaximumMetersPerSecond;

  private TalonSwerveModule(Builder builder) {
    this.azimuthTalon = builder.azimuthTalon;
    this.driveTalon = builder.driveTalon;
    this.azimuthCountsPerRev = builder.azimuthCountsPerRev;
    this.driveCountsPerRev = builder.driveCountsPerRev;
    driveGearRatio = builder.driveGearRatio;
    this.wheelCircumferenceMeters = Math.PI * Units.inchesToMeters(builder.wheelDiameterInches);
    this.driveMaximumMetersPerSecond = builder.driveMaximumMetersPerSecond;
  }

  @Override
  public SwerveModuleState getState() {
    double speedMetersPerSecond = getDriveMetersPerSecond();
    Rotation2d angle = getAzimuthRotation2d();
    return new SwerveModuleState(speedMetersPerSecond, angle);
  }

  @Override
  public void setOpenLoopDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentAngle = getAzimuthRotation2d();
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);
    setAzimuthRotation2d(optimizedState.angle);
    setDriveOpenLoopMetersPerSecond(optimizedState.speedMetersPerSecond);
  }

  @Override
  public void setClosedLoopDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentAngle = getAzimuthRotation2d();
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);
    setAzimuthRotation2d(optimizedState.angle);
    setDriveClosedLoopMetersPerSecond(optimizedState.speedMetersPerSecond);
  }

  private Rotation2d getAzimuthRotation2d() {
    double azimuthRawCounts = azimuthTalon.getSelectedSensorPosition();
    double azimuthCounts = Math.IEEEremainder(azimuthRawCounts, azimuthCountsPerRev);
    double radians = 2.0 * Math.PI * azimuthCounts / azimuthCountsPerRev;
    return new Rotation2d(radians);
  }

  private void setAzimuthRotation2d(Rotation2d angle) {
    double azimuthRawCounts = azimuthTalon.getSelectedSensorPosition();
    double azimuthCountsChange = angle.getRadians() / (2.0 * Math.PI);
    azimuthTalon.set(ControlMode.MotionMagic, azimuthRawCounts + azimuthCountsChange);
  }

  private double getDriveMetersPerSecond() {
    double encoderCountsPer100ms = driveTalon.getSelectedSensorVelocity();
    double motorRotationsPer100ms = encoderCountsPer100ms / driveCountsPerRev;
    double wheelRotationsPer100ms = motorRotationsPer100ms * driveGearRatio;
    double metersPer100ms = wheelRotationsPer100ms * wheelCircumferenceMeters;
    return metersPer100ms * k100msPerSecond;
  }

  private void setDriveOpenLoopMetersPerSecond(double metersPerSecond) {
    driveTalon.set(ControlMode.PercentOutput, metersPerSecond / driveMaximumMetersPerSecond);
  }

  private void setDriveClosedLoopMetersPerSecond(double metersPerSecond) {
    double wheelRotationsPerSecond = metersPerSecond / wheelCircumferenceMeters;
    double motorRotationsPerSecond = wheelRotationsPerSecond / driveGearRatio;
    double encoderCountsPerSecond = motorRotationsPerSecond * driveCountsPerRev;
    driveTalon.set(ControlMode.Velocity, encoderCountsPerSecond / k100msPerSecond);
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
    private double driveMaximumMetersPerSecond;

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

    public Builder driveMaximumMetersPerSecond(double metersPerSecond) {
      driveMaximumMetersPerSecond = metersPerSecond;
      return this;
    }

    public TalonSwerveModule build() {
      return new TalonSwerveModule(this);
    }
  }
}
