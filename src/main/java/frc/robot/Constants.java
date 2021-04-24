// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

  public static final class Drive {

    public static final double kDriveMotorOutputGear = 22;
    public static final double kDriveInputGear = 48;
    public static final double kBevelInputGear = 15;
    public static final double kBevelOutputGear = 45;
    public static final double kDriveGearRatio =
        (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);

    public static final double kWheelDiameterInches = 3;
    public static final double kDriveMaximumMetersPerSecond = 3.84020432; // 6300 rpm

    private final static double kWheelOffsetMeters = 0.27305;
    public static final Translation2d kLeftFrontWheelLocation = new Translation2d(
        kWheelOffsetMeters, kWheelOffsetMeters);
    public static final Translation2d kRightFrontWheelLocation = new Translation2d(
        kWheelOffsetMeters, -kWheelOffsetMeters);
    public static final Translation2d kLeftRearWheelLocation = new Translation2d(
        -kWheelOffsetMeters, kWheelOffsetMeters);
    public static final Translation2d kRightRearWheelLocation = new Translation2d(
        -kWheelOffsetMeters, -kWheelOffsetMeters);

    public final static TalonSRXConfiguration kAzimuthTalonConfiguration = new TalonSRXConfiguration();
    public final static TalonFXConfiguration kDriveTalonConfiguration = new TalonFXConfiguration();

    static {
      kAzimuthTalonConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
      kAzimuthTalonConfiguration.continuousCurrentLimit = 10;
      kAzimuthTalonConfiguration.peakCurrentDuration = 0;
      kAzimuthTalonConfiguration.peakCurrentLimit = 0;
      kAzimuthTalonConfiguration.slot0.kP = 10.0;
      kAzimuthTalonConfiguration.slot0.kI = 0.0;
      kAzimuthTalonConfiguration.slot0.kD = 100.0;
      kAzimuthTalonConfiguration.slot0.kF = 0.0;
      kAzimuthTalonConfiguration.slot0.integralZone = 0;
      kAzimuthTalonConfiguration.slot0.allowableClosedloopError = 0;
      kAzimuthTalonConfiguration.motionAcceleration = 10_000;
      kAzimuthTalonConfiguration.motionCruiseVelocity = 800;
      kAzimuthTalonConfiguration.velocityMeasurementWindow = 64;
      kAzimuthTalonConfiguration.voltageCompSaturation = 12;

      kDriveTalonConfiguration.supplyCurrLimit.currentLimit = 0.04;
      kDriveTalonConfiguration.supplyCurrLimit.triggerThresholdCurrent = 45;
      kDriveTalonConfiguration.supplyCurrLimit.triggerThresholdTime = 40;
      kDriveTalonConfiguration.supplyCurrLimit.enable = true;
      kDriveTalonConfiguration.slot0.kP = 0.045;
      kDriveTalonConfiguration.slot0.kI = 0.0005;
      kDriveTalonConfiguration.slot0.kD = 0.000;
      kDriveTalonConfiguration.slot0.kF = 0.047;
      kDriveTalonConfiguration.slot0.integralZone = 500;
      kDriveTalonConfiguration.slot0.maxIntegralAccumulator = 75_000;
      kDriveTalonConfiguration.slot0.allowableClosedloopError = 0;
      kDriveTalonConfiguration.velocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms;
      kDriveTalonConfiguration.velocityMeasurementWindow = 64;
      kDriveTalonConfiguration.voltageCompSaturation = 12;
    }

  }
}
