// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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

  public static final class Drive {

    public static final double kDriveMotorOutputGear = 22;
    public static final double kDriveInputGear = 48;
    public static final double kBevelInputGear = 15;
    public static final double kBevelOutputGear = 45;
    public static final double kDriveGearRatio =
        (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);

    // Skippy
    public static final double kWheelDiameterInches = 2.4;
    public static final double kMaxSpeedMetersPerSecond = 3.53568;

    public final static Translation2d[] kWheelLocations = new Translation2d[4];

    public final static TalonSRXConfiguration kAzimuthTalonConfiguration;
    public final static TalonSRXConfiguration kDriveTalonConfiguration = new TalonSRXConfiguration();

    static {
      final double offset = 0.27305;
      kWheelLocations[0] = new Translation2d(offset, offset); // left front
      kWheelLocations[1] = new Translation2d(offset, -offset); // right front
      kWheelLocations[2] = new Translation2d(-offset, offset); // left rear
      kWheelLocations[3] = new Translation2d(-offset, -offset); // right rear
    }

    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#closed-loop-configurations
    static {
      // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
      kAzimuthTalonConfiguration = new TalonSRXConfiguration();

      kAzimuthTalonConfiguration.primaryPID.selectedFeedbackCoefficient = 1.0;
      kAzimuthTalonConfiguration.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;

      kAzimuthTalonConfiguration.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      kAzimuthTalonConfiguration.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;

      kAzimuthTalonConfiguration.continuousCurrentLimit = 10;
      kAzimuthTalonConfiguration.peakCurrentDuration = 1;
      kAzimuthTalonConfiguration.peakCurrentLimit = 1;
      kAzimuthTalonConfiguration.slot0.kP = 10.0;
      kAzimuthTalonConfiguration.slot0.kI = 0.0;
      kAzimuthTalonConfiguration.slot0.kD = 100.0;
      kAzimuthTalonConfiguration.slot0.kF = 1.0;
      kAzimuthTalonConfiguration.slot0.integralZone = 0;
      kAzimuthTalonConfiguration.slot0.allowableClosedloopError = 0;
      kAzimuthTalonConfiguration.slot0.maxIntegralAccumulator = 10;
      kAzimuthTalonConfiguration.motionCruiseVelocity = 800;
      kAzimuthTalonConfiguration.motionAcceleration = 10_000;
      kAzimuthTalonConfiguration.velocityMeasurementWindow = 64;
      kAzimuthTalonConfiguration.voltageCompSaturation = 12;

      kDriveTalonConfiguration.continuousCurrentLimit = 40;
      kDriveTalonConfiguration.peakCurrentDuration = 0;
      kDriveTalonConfiguration.peakCurrentLimit = 0;
      kDriveTalonConfiguration.slot0.kP = 0.010;
      kDriveTalonConfiguration.slot0.kI = 0.0003;
      kDriveTalonConfiguration.slot0.kD = 0.600;
      kDriveTalonConfiguration.slot0.kF = 0.028;
      kDriveTalonConfiguration.slot0.integralZone = 3000;
      kDriveTalonConfiguration.slot0.maxIntegralAccumulator = 200_000;
      kDriveTalonConfiguration.slot0.allowableClosedloopError = 0;
      kDriveTalonConfiguration.velocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms;
      kDriveTalonConfiguration.velocityMeasurementWindow = 64;
      kDriveTalonConfiguration.voltageCompSaturation = 12;
    }

  }
}
