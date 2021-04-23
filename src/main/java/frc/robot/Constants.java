// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }
}
