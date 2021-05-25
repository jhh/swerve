package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import org.strykeforce.trapper.Action;
import org.strykeforce.trapper.Activity;
import org.strykeforce.trapper.PostCommand;
import org.strykeforce.trapper.TrapperSubsystem;

public class ActivityCommandGroup extends SequentialCommandGroup {

  private final DriveSubsystem driveSubsystem;
  private final TrapperSubsystem trapperSubsystem;
  private final DriveTrajectoryCommand driveTrajectoryCommand;
  private Activity activity = new Activity("ActivityCommandGroup");
  private Action action;


  public ActivityCommandGroup(TrapperSubsystem trapperSubsystem, DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.trapperSubsystem = trapperSubsystem;

    addRequirements(trapperSubsystem, driveSubsystem);
    driveTrajectoryCommand = createDriveTrajectoryCommand();
    addCommands(driveTrajectoryCommand);

    if (trapperSubsystem.isEnabled()) {
      activity.getMeta().put("description", "Jif trajectory following");
      addCommands(
          new PostCommand(trapperSubsystem,
              () -> trapperSubsystem.postAsync(activity),
              interrupted -> activity = trapperSubsystem.getActivity())
          ,
          new PostCommand(trapperSubsystem,
              () -> {
                action = driveTrajectoryCommand.getAction();
                action.setActivity(activity.getUrl());
                trapperSubsystem.postAsync(action);
              },
              interrupted -> action = trapperSubsystem.getAction()),
          new PostCommand(trapperSubsystem,
              () -> {
                var traces = driveTrajectoryCommand.getTraces();
                traces.forEach(t -> t.setAction(action.getId()));
                trapperSubsystem.post(traces);
              },
              interrupted -> {
              })
      );
    }
  }

  public DriveTrajectoryCommand getDriveTrajectoryCommand() {
    return driveTrajectoryCommand;
  }

  private DriveTrajectoryCommand createDriveTrajectoryCommand() {
    var config = new TrajectoryConfig(3, 6);
    config.setKinematics(driveSubsystem.getSwerveDriveKinematics());

    Pose2d start = new Pose2d(0, 0, new Rotation2d());
    List<Translation2d> waypoints = Arrays.asList(new Translation2d(1, 0.5), new Translation2d(2, -0.5));
//    Pose2d end = new Pose2d(3, 0, new Rotation2d());
//    List<Translation2d> waypoints = Collections.singletonList(new Translation2d(1, 0));
    Pose2d end = new Pose2d(3, 0, new Rotation2d());

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

    return new DriveTrajectoryCommand(driveSubsystem, trapperSubsystem, trajectory, meta);
  }
}
