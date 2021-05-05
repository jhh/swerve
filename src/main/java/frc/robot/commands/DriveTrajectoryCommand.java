package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.trapper.Action;
import org.strykeforce.trapper.Session;
import org.strykeforce.trapper.Trace;

public class DriveTrajectoryCommand extends CommandBase {

  private static final Logger logger = LoggerFactory.getLogger(DriveTrajectoryCommand.class);

  private final DriveSubsystem driveSubsystem;
  private final Trajectory trajectory;
  private final Map<String, Object> meta;
  private final Timer timer = new Timer();
  private HolonomicDriveController holonomicDriveController;
  private Trapper trapper;

  public DriveTrajectoryCommand(DriveSubsystem driveSubsystem, Trajectory trajectory,
      Map<String, Object> meta) {
    this.driveSubsystem = driveSubsystem;
    this.trajectory = trajectory;
    this.meta = meta;
    hasRequirement(driveSubsystem);
    logger.info("loaded trajectory with total time = {} sec", trajectory.getTotalTimeSeconds());
  }

  @Override
  public void initialize() {
    holonomicDriveController = new HolonomicDriveController(
        new PIDController(2, 0, 0), new PIDController(2, 0, 0),
        new ProfiledPIDController(2, 0, 0,
            new TrapezoidProfile.Constraints(6.28, 3.14)));

    holonomicDriveController.setEnabled(true);

    if (Trapper.isEnabled) {
      trapper = new Trapper("http://192.168.3.169:8000", meta, driveSubsystem);
    }
    driveSubsystem.resetOdometry(trajectory.getInitialPose());
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    var state = trajectory.sample(timer.get());
    logger.debug("state = {}", state);
    var speeds = holonomicDriveController
        .calculate(driveSubsystem.getPoseMeters(), state, new Rotation2d());
    driveSubsystem.move(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond);
    if (Trapper.isEnabled) {
      trapper.logState(state, speeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0);

    if (Trapper.isEnabled) {
      trapper.post();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  private static class Trapper {
    final static boolean isEnabled = true;

    final DriveSubsystem driveSubsystem;
    final List<Trace> traces = new ArrayList<>();
    final Session session;
    int actionId;

    Trapper(String url, Map<String, Object> meta, DriveSubsystem driveSubsystem) {
      this.driveSubsystem = driveSubsystem;
      session = new Session(url);
      meta.put("command", DriveTrajectoryCommand.class.getSimpleName());
      Action action = new Action(meta.get("name").toString());
      action.setMeta(meta);
      var measures = action.getMeasures();
      measures.add("traj_accel");
      measures.add("traj_curvature");
      measures.add("traj_pose_x");
      measures.add("traj_pose_y");
      measures.add("traj_pose_degrees");
      measures.add("traj_time");
      measures.add("traj_vel");
      measures.add("gyro_degrees");
      measures.add("hc_vx");
      measures.add("hc_vy");
      measures.add("hc_omega");
      measures.add("od_pose_x");
      measures.add("od_pose_y");
      measures.add("od_pose_degrees");


      action = session.post(action);
      if (action.getId() == null) {
        logger.error("action failed to post - id was null");
        return;
      }
      actionId = action.getId();
    }

    void logState(Trajectory.State state, ChassisSpeeds speeds) {
      var trace = new Trace((int) Math.round(state.timeSeconds * 1000));
      trace.setAction(actionId);
      traces.add(trace);
      var data = trace.getData();
      data.add(state.accelerationMetersPerSecondSq);
      data.add(state.curvatureRadPerMeter);
      data.add(state.poseMeters.getX());
      data.add(state.poseMeters.getY());
      data.add(state.poseMeters.getRotation().getDegrees());
      data.add(state.timeSeconds);
      data.add(state.velocityMetersPerSecond);
      data.add(driveSubsystem.getHeading().getDegrees());
      data.add(speeds.vxMetersPerSecond);
      data.add(speeds.vyMetersPerSecond);
      data.add(speeds.omegaRadiansPerSecond);
      data.add(driveSubsystem.getPoseMeters().getX());
      data.add(driveSubsystem.getPoseMeters().getY());
      data.add(driveSubsystem.getPoseMeters().getRotation().getDegrees());
    }

    void post() {
      session.post(traces);
    }

  }
}
