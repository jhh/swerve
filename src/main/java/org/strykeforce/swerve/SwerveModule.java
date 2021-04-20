package org.strykeforce.swerve;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public interface SwerveModule {

    /** Gets the current state of the swerve module. */
    SwerveModuleState getState();

    /** Command the swerve module motors to the desired state. */
    void setDesiredState(SwerveModuleState desiredState);

}
