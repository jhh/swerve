package org.strykeforce.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
  private final Translation2d wheelLocationMeters;

  private TalonSwerveModule(Builder builder) {
    azimuthTalon = builder.azimuthTalon;
    driveTalon = builder.driveTalon;
    azimuthCountsPerRev = builder.azimuthCountsPerRev;
    driveCountsPerRev = builder.driveCountsPerRev;
    driveGearRatio = builder.driveGearRatio;
    wheelCircumferenceMeters = Math.PI * Units.inchesToMeters(builder.wheelDiameterInches);
    driveMaximumMetersPerSecond = builder.driveMaximumMetersPerSecond;
    wheelLocationMeters = builder.wheelLocationMeters;
  }

  @Override
  public Translation2d getWheelLocationMeters() {
    return wheelLocationMeters;
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

  @Override
  public void storeAzimuthZeroReference() {
    String preferenceKey = getKey();
//    azimuthTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
  }


  private Rotation2d getAzimuthRotation2d() {
    double azimuthCounts = azimuthTalon.getSelectedSensorPosition();
    double radians = 2.0 * Math.PI * azimuthCounts / azimuthCountsPerRev;
    return new Rotation2d(radians);
  }

  private void setAzimuthRotation2d(Rotation2d angle) {
    double countsBefore = azimuthTalon.getSelectedSensorPosition();
    double countsFromAngle = angle.getRadians() / (2.0 * Math.PI) * azimuthCountsPerRev;
    double countsDelta = Math.IEEEremainder(countsFromAngle - countsBefore, azimuthCountsPerRev);
    azimuthTalon.set(ControlMode.MotionMagic, countsBefore + countsDelta);
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

  private String getKey() {
    if (wheelLocationMeters.getX() > 0 && wheelLocationMeters.getY() < 0) {
      return "1FR";
    }
    if (wheelLocationMeters.getX() > 0 && wheelLocationMeters.getY() > 0) {
      return "2FL";
    }
    if (wheelLocationMeters.getX() < 0 && wheelLocationMeters.getY() < 0) {
      return "3RR";
    }
    return "4RL";
  }

  @Override
  public String toString() {
    return "TalonSwerveModule{" + getKey() + '}';
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
    private Translation2d wheelLocationMeters;

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

    public Builder wheelLocationMeters(Translation2d locationMeters) {
      wheelLocationMeters = locationMeters;
      return this;
    }

    public TalonSwerveModule build() {
      var module = new TalonSwerveModule(this);
      validateTalonSwerveModuleObject(module);
      return module;
    }


    private void validateTalonSwerveModuleObject(TalonSwerveModule module) {
      if (module.driveGearRatio <= 0) {
        throw new IllegalArgumentException("drive gear ratio must be greater than zero.");
      }

      if (module.wheelCircumferenceMeters <= 0) {
        throw new IllegalArgumentException("wheel diameter must be greater than zero.");
      }

      if (module.driveMaximumMetersPerSecond <= 0) {
        throw new IllegalArgumentException("drive maximum speed must be greater than zero.");
      }

      if (module.wheelLocationMeters == null) {
        throw new IllegalArgumentException("wheel location must be set.");
      }
    }
  }
}
