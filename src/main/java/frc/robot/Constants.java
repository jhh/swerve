// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final static int kTalonConfigTimeout = 10; // ms

  public static final class DriveConstants {

    // Skippy
    static final double kDriveMotorOutputGear = 25;
    static final double kDriveInputGear = 55;
    static final double kBevelInputGear = 20;
    static final double kBevelOutputGear = 40;
    public static final double kDriveGearRatio =
        (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);

    public static final double kWheelDiameterInches = 2.4 * 0.497;
    public static final double kMaxSpeedMetersPerSecond = 3.53568;


    public static Translation2d[] getWheelLocationMeters() {
      // skippy is square frame
      final double offset = 0.27305;
      Translation2d[] locs = new Translation2d[4];
      locs[0] = new Translation2d(offset, offset); // left front
      locs[1] = new Translation2d(offset, -offset); // right front
      locs[2] = new Translation2d(-offset, offset); // left rear
      locs[3] = new Translation2d(-offset, -offset); // right rear
      return locs;
    }

    public static TalonSRXConfiguration getAzimuthTalonConfig() {
      // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
      TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();

      azimuthConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
      azimuthConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

      azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

      azimuthConfig.continuousCurrentLimit = 10;
      azimuthConfig.peakCurrentDuration = 1;
      azimuthConfig.peakCurrentLimit = 1;
      azimuthConfig.slot0.kP = 10.0;
      azimuthConfig.slot0.kI = 0.0;
      azimuthConfig.slot0.kD = 100.0;
      azimuthConfig.slot0.kF = 1.0;
      azimuthConfig.slot0.integralZone = 0;
      azimuthConfig.slot0.allowableClosedloopError = 0;
      azimuthConfig.slot0.maxIntegralAccumulator = 10;
      azimuthConfig.motionCruiseVelocity = 800;
      azimuthConfig.motionAcceleration = 10_000;
      azimuthConfig.velocityMeasurementWindow = 64;
      azimuthConfig.voltageCompSaturation = 12;
      return azimuthConfig;
    }

    public static TalonSRXConfiguration getDriveTalonConfig() {
      TalonSRXConfiguration driveConfig = new TalonSRXConfiguration();
      driveConfig.continuousCurrentLimit = 40;
      driveConfig.peakCurrentDuration = 0;
      driveConfig.peakCurrentLimit = 0;
      driveConfig.slot0.kP = 0.010;
      driveConfig.slot0.kI = 0.0003;
      driveConfig.slot0.kD = 0.600;
      driveConfig.slot0.kF = 0.028;
      driveConfig.slot0.integralZone = 3000;
      driveConfig.slot0.maxIntegralAccumulator = 200_000;
      driveConfig.slot0.allowableClosedloopError = 0;
      driveConfig.velocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms;
      driveConfig.velocityMeasurementWindow = 64;
      driveConfig.voltageCompSaturation = 12;
      return driveConfig;
    }

  }
}
