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
   * Command the swerve module motors to the desired state.
   *
   * @param desiredState      the desired swerve module speed and angle
   * @param isDriveOpenLoop true if drive should set speed using closed-loop velocity control
   */
  void setDesiredState(SwerveModuleState desiredState, boolean isDriveOpenLoop);

  /**
   * Command the swerve module motors to the desired state using closed-loop drive speed control.
   *
   * @param desiredState the desired swerve module speed and angle
   */
  default void setDesiredState(SwerveModuleState desiredState) {
    this.setDesiredState(desiredState, false);
  }

  /**
   * Save the current azimuth absolute encoder reference position in NetworkTables preferences. Call
   * this method following physical alignment of the module wheel in its zeroed position. Used
   * during module instantiation to initialize the relative encoder.
   */
  void storeAzimuthZeroReference();

  /**
   * Loads the current azimuth absolute encoder reference position and sets selected sensor
   * encoder.
   */
  void loadAndSetAzimuthZeroReference();
}
