package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TimedDriveCommand extends CommandBase {

  private static final Logger logger = LoggerFactory.getLogger(TimedDriveCommand.class);

  private final DriveSubsystem driveSubsystem;
  private final Timer timer = new Timer();

  public TimedDriveCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.move(2.0, 0.0, 0.0, false);
    logger.debug("initialize()");
    timer.reset();
    timer.start();
  }

//  @Override
//  public void execute() {
//    driveSubsystem.move(2.0, 0.0, 0.0, false);
//  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.move(0.0, 0.0, 0.0, false);
    logger.debug("end()");
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2.0);
  }
}
