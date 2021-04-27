package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class IndexAzimuthCommand extends InstantCommand {

  private final DriveSubsystem driveSubsystem;

  public IndexAzimuthCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    hasRequirement(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.next();
  }
}
