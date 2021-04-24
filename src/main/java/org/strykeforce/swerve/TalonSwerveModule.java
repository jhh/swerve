package org.strykeforce.swerve;

import static com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import java.util.Objects;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TalonSwerveModule implements SwerveModule {

  private static final Logger logger = LoggerFactory.getLogger(TalonSwerveModule.class);

  final int k100msPerSecond = 10;

  private final TalonSRX azimuthTalon;
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
  public void setDesiredState(SwerveModuleState desiredState, boolean isDriveOpenLoop) {
    Rotation2d currentAngle = getAzimuthRotation2d();
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);
    setAzimuthRotation2d(optimizedState.angle);
    if (isDriveOpenLoop) {
      setDriveOpenLoopMetersPerSecond(optimizedState.speedMetersPerSecond);
    } else {
      setDriveClosedLoopMetersPerSecond(optimizedState.speedMetersPerSecond);
    }
  }

  @Override
  public void storeAzimuthZeroReference() {
    int index = getWheelIndex();
    int position = getAzimuthAbsoluteEncoderCounts();
    Preferences preferences = Preferences.getInstance();
    String key = String.format("SwerveDrive/wheel.%d", index);
    preferences.putInt(key, position);
    logger.info("azimuth {}: saved zero = {}", index, position);
  }

  @Override
  public void loadAndSetAzimuthZeroReference() {
    int index = getWheelIndex();
    Preferences preferences = Preferences.getInstance();
    String key = String.format("SwerveDrive/wheel.%d", index);
    int reference = preferences.getInt(key, Integer.MIN_VALUE);
    if (reference == Integer.MIN_VALUE) {
      logger.error("no saved azimuth zero reference for swerve module {}", index);
      throw new IllegalStateException();
    }
    logger.info("swerve module {}: loaded azimuth zero reference = {}", index, reference);

    int azimuthAbsoluteCounts = getAzimuthAbsoluteEncoderCounts();
    logger.info("swerve module {}: azimuth absolute position = {}", index, azimuthAbsoluteCounts);


    int azimuthSetpoint = azimuthAbsoluteCounts - reference;
    ErrorCode err = azimuthTalon.setSelectedSensorPosition(azimuthSetpoint, 0, 10);
    if (err.value != 0) {
      logger.warn("Talon error code while setting azimuth zero: {}", err);
    }

    azimuthTalon.set(MotionMagic, azimuthSetpoint);
    logger.info("swerve module {}: set azimuth encoder = {}", index, azimuthSetpoint);
  }


  private int getAzimuthAbsoluteEncoderCounts() {
    return azimuthTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
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
    azimuthTalon.set(MotionMagic, countsBefore + countsDelta);
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

  private int getWheelIndex() {
    if (wheelLocationMeters.getX() > 0 && wheelLocationMeters.getY() > 0) {
      return 0;
    }
    if (wheelLocationMeters.getX() > 0 && wheelLocationMeters.getY() < 0) {
      return 1;
    }
    if (wheelLocationMeters.getX() < 0 && wheelLocationMeters.getY() > 0) {
      return 2;
    }
    return 3;
  }

  @Override
  public String toString() {
    return "TalonSwerveModule{" + getWheelIndex() + '}';
  }

  public static class Builder {

    public static final int kDefaultTalonSRXCountsPerRev = 4096;
    public static final int kDefaultTalonFXCountsPerRev = 2048;

    private final TalonSRX azimuthTalon;
    private final BaseTalon driveTalon;
    private final int azimuthCountsPerRev = kDefaultTalonSRXCountsPerRev;
    private double driveGearRatio;
    private double wheelDiameterInches;
    private int driveCountsPerRev = kDefaultTalonFXCountsPerRev;
    private double driveMaximumMetersPerSecond;
    private Translation2d wheelLocationMeters;

    public Builder(TalonSRX azimuthTalon, BaseTalon driveTalon) {
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

    // we currently only support TalonSRX for azimuth
//    public Builder azimuthEncoderCountsPerRevolution(int countsPerRev) {
//      azimuthCountsPerRev = countsPerRev;
//      return this;
//    }

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

      if (module.azimuthCountsPerRev <= 0) {
        throw new IllegalArgumentException(
            "azimuth encoder counts per revolution must be greater than zero.");
      }

      if (module.driveCountsPerRev <= 0) {
        throw new IllegalArgumentException(
            "drive encoder counts per revolution must be greater than zero.");
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
