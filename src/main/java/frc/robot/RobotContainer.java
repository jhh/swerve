// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.subsystems.ConsoleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static final Logger logger = LoggerFactory.getLogger(RobotContainer.class);

  private final static double kJoystickDeadband = 0.1;

  // The robot's subsystems and commands are defined here...
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);
  private final ConsoleSubsystem consoleSubsystem = new ConsoleSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(telemetryService,
      consoleSubsystem);
  private final Joystick joystick = new Joystick(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
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
    telemetryService.start();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Button userButton = new Button() {
      @Override
      public boolean get() {
        return RobotController.getUserButton();
      }
    };
    userButton.whenPressed(new InstantCommand(consoleSubsystem::toggle, consoleSubsystem) {
      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });
    new JoystickButton(joystick, InterlinkButton.X.id)
        .whenPressed(getLogTrajectoryCommand());

    new JoystickButton(joystick, InterlinkButton.RESET.id)
        .whenPressed(driveSubsystem::resetGyro, driveSubsystem);

    new JoystickButton(joystick, InterlinkButton.HAMBURGER.id)
        .whenPressed(() -> {
              logger.debug("pose = {}", driveSubsystem.getPoseMeters());
              driveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
            },
            driveSubsystem);
  }

  private Command getLogTrajectoryCommand() {
    var config = new TrajectoryConfig(2, 4);
    config.setKinematics(driveSubsystem.getSwerveDriveKinematics());

    Pose2d start = new Pose2d(0, 0, new Rotation2d());
//    List<Translation2d> waypoints = Arrays.asList(new Translation2d(1, 1), new Translation2d(2, -1));
//    Pose2d end = new Pose2d(3, 0, new Rotation2d());
    List<Translation2d> waypoints = Collections.singletonList(new Translation2d(1, 0));
    Pose2d end = new Pose2d(2, 0, new Rotation2d());

    var trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

    var meta = new HashMap<String, Object>();
    meta.put("name", "Trajectory Testing");
    meta.put("description", "2.0m x-direction trajectory");
    meta.put("version", "a30f0b6");
    meta.put("simulator", Boolean.FALSE);
    meta.put("trajectoryTime", trajectory.getTotalTimeSeconds());
    var trajectoryMeta = new HashMap<String, Object>();
    trajectoryMeta.put("startPose", start);
    trajectoryMeta.put("waypoints", waypoints);
    trajectoryMeta.put("endPose", end);
    meta.put("trajectory", trajectoryMeta);
    return new DriveTrajectoryCommand(driveSubsystem, trajectory, meta);
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
