package org.strykeforce.swerve;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public interface SwerveModule {

  /**
   * Provides the wheel location as Translation2d.
   */
  Translation2d getWheelLocationMeters();

  /**
   * Gets the current state of the swerve module.
   */
  SwerveModuleState getState();

  /**
   * Command the swerve module motors to the desired state using open loop drive velocity.
   */
  void setOpenLoopDesiredState(SwerveModuleState desiredState);

  /**
   * Command the swerve module motors to the desired state using closed loop drive velocity.
   */
  void setClosedLoopDesiredState(SwerveModuleState desiredState);

  /**
   * Save the current azimuth absolute encoder position in NetworkTables preferences. Call this
   * method following physical alignment of the module wheel in its zeroed position. Used during
   * module instantiation to initialize the relative encoder.
   */
  void storeAzimuthZeroReference();
}
