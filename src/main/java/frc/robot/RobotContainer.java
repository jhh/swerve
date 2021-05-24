// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ActivityCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.console.ConsoleSubsystem;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.trapper.TrapperSubsystem;


public class RobotContainer {

  private static final Logger logger = LoggerFactory.getLogger(RobotContainer.class);

  private final static double kJoystickDeadband = 0.1;

  // The robot's subsystems and commands are defined here...
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);
  private final ConsoleSubsystem consoleSubsystem = new ConsoleSubsystem(false);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(telemetryService);
  private final TrapperSubsystem trapperSubsystem = new TrapperSubsystem("http://192.168.3.3:3003",
      false);
  private final Joystick joystick = new Joystick(0);
  private final ActivityCommandGroup activityCommandGroup = new ActivityCommandGroup(
      trapperSubsystem, driveSubsystem);


  public RobotContainer() {
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(new RunCommand(
        () -> {
          double vx = getLeftX() * -DriveConstants.kMaxSpeedMetersPerSecond;
          double vy = getLeftY() * -DriveConstants.kMaxSpeedMetersPerSecond;
          double omega = getRightY() * DriveConstants.kMaxOmega;
          driveSubsystem.drive(vx, vy, omega);
        }
        , driveSubsystem));

    telemetryService.register(driveSubsystem);
    telemetryService.register(activityCommandGroup.getDriveTrajectoryCommand());
    telemetryService.start();
  }


  private void configureButtonBindings() {
    new Button(RobotController::getUserButton).whenPressed(new PrintCommand("user button pressed"));

    new JoystickButton(joystick, InterlinkButton.X.id).whenPressed(activityCommandGroup);

    new JoystickButton(joystick, InterlinkButton.RESET.id)
        .whenPressed(driveSubsystem::resetGyro, driveSubsystem);

    new JoystickButton(joystick, InterlinkButton.HAMBURGER.id)
        .whenPressed(() -> {
              logger.debug("pose = {}", driveSubsystem.getPoseMeters());
              driveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
            },
            driveSubsystem);

    var entry = NetworkTableInstance.getDefault().getEntry("Trigger");
    new NetworkButton(entry)
        .whenPressed(activityCommandGroup.andThen(() -> entry.setBoolean(false), driveSubsystem));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  /**
   * Left stick X (up-down) axis.
   */
  public double getLeftX() {
    double val = joystick.getRawAxis(Axis.LEFT_X.id);
    if (Math.abs(val) < kJoystickDeadband) {
      return 0.0;
    }
    return val;
  }

  /**
   * Left stick Y (left-right) axis.
   */
  public double getLeftY() {
    double val = joystick.getRawAxis(Axis.LEFT_Y.id);
    if (Math.abs(val) < kJoystickDeadband) {
      return 0.0;
    }
    return val;
  }

  /**
   * Right stick Y (left-right) axis.
   */
  public double getRightY() {
    double val = joystick.getRawAxis(Axis.RIGHT_Y.id);
    if (Math.abs(val) < kJoystickDeadband) {
      return 0.0;
    }
    return val;
  }

  public enum Axis {
    RIGHT_X(1),
    RIGHT_Y(0),
    LEFT_X(2),
    LEFT_Y(5),
    TUNER(6),
    LEFT_BACK(4),
    RIGHT_BACK(3);

    private final int id;

    Axis(int id) {
      this.id = id;
    }
  }

  public enum Shoulder {
    RIGHT_DOWN(2),
    LEFT_DOWN(4),
    LEFT_UP(5);

    private final int id;

    Shoulder(int id) {
      this.id = id;
    }
  }

  public enum Toggle {
    LEFT_TOGGLE(1);

    private final int id;

    Toggle(int id) {
      this.id = id;
    }
  }

  public enum InterlinkButton {
    RESET(3),
    HAMBURGER(14),
    X(15),
    UP(16),
    DOWN(17);

    private final int id;

    InterlinkButton(int id) {
      this.id = id;
    }
  }

  public enum Trim {
    LEFT_Y_POS(7),
    LEFT_Y_NEG(6),
    LEFT_X_POS(8),
    LEFT_X_NEG(9),
    RIGHT_X_POS(10),
    RIGHT_X_NEG(11),
    RIGHT_Y_POS(12),
    RIGHT_Y_NEG(13);

    private final int id;

    Trim(int id) {
      this.id = id;
    }
  }
}
