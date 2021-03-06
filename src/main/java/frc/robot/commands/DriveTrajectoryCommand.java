package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Map;
import java.util.Set;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.Measurable;
import org.strykeforce.telemetry.measurable.Measure;
import org.strykeforce.trapper.ActionCommand;
import org.strykeforce.trapper.Trace;
import org.strykeforce.trapper.TrapperSubsystem;

public class DriveTrajectoryCommand extends ActionCommand implements Measurable {

  private static final Logger logger = LoggerFactory.getLogger(DriveTrajectoryCommand.class);

  private final DriveSubsystem driveSubsystem;
  private final Trajectory trajectory;
  private final Timer timer = new Timer();
  private boolean isTimerStarted;
  private HolonomicDriveController holonomicDriveController;
  private Trajectory.State state = new State();
  private Pose2d odometryPose = new Pose2d();
  private ChassisSpeeds speeds = new ChassisSpeeds();

  public DriveTrajectoryCommand(DriveSubsystem driveSubsystem, TrapperSubsystem trapperSubsystem,
      Trajectory trajectory, Map<String, Object> meta) {
    super(trapperSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.trajectory = trajectory;
    logger.info("loaded trajectory with total time = {} sec", trajectory.getTotalTimeSeconds());

    addRequirements(driveSubsystem, trapperSubsystem);

    if (trapperSubsystem.isEnabled()) {
      getAction().getMeta().putAll(meta);
      var measures = getAction().getMeasures();
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
    }
    timer.start();
  }

  @Override
  public void initialize() {
    var p = 6.0;
    var d = p / 100.0;
    holonomicDriveController = new HolonomicDriveController(
        new PIDController(p, 0, d), new PIDController(p, 0, d),
        new ProfiledPIDController(-2.5, 0, 0,
            new TrapezoidProfile.Constraints(DriveConstants.kMaxOmega / 2.0, 3.14)));

    holonomicDriveController.setEnabled(true);

    driveSubsystem.resetOdometry(trajectory.getInitialPose());
    timer.reset();
//    isTimerStarted = false;
//    timer.start();
  }

  @Override
  public void execute() {
//    if (!isTimerStarted) {
//      isTimerStarted = true;
//      timer.start();
//    }
    state = trajectory.sample(timer.get());
    odometryPose = driveSubsystem.getPoseMeters();
    speeds = holonomicDriveController.calculate(odometryPose, state, new Rotation2d());
    driveSubsystem.move(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond, true);
    super.execute();
  }

  @NotNull
  @Override
  public Trace getTrace() {
    if (!getTrapperSubsystem().isEnabled()) {
      throw new IllegalStateException("Trapper subsystem is not enabled");
    }

    var trace = new Trace((int) Math.round(state.timeSeconds * 1000));
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
    data.add(odometryPose.getX());
    data.add(odometryPose.getY());
    data.add(odometryPose.getRotation().getDegrees());

    return trace;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @NotNull
  @Override
  public String getDescription() {
    return "Trajectory Command";
  }

  @Override
  public int getDeviceId() {
    return 0;
  }

  @NotNull
  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Traj. Accel", () -> state.accelerationMetersPerSecondSq),
        new Measure("Traj. Curvature", () -> state.curvatureRadPerMeter),
        new Measure("Traj. X", () -> state.poseMeters.getX()),
        new Measure("Traj. Y", () -> state.poseMeters.getY()),
        new Measure("Traj. Degrees", () -> state.poseMeters.getRotation().getDegrees()),
        new Measure("Traj. Time", () -> state.timeSeconds),
        new Measure("Traj. Vel", () -> state.velocityMetersPerSecond),
//        new Measure("Gyro Degrees", () -> driveSubsystem.getHeading().getDegrees()),
        new Measure("HC Vx", () -> speeds.vxMetersPerSecond),
        new Measure("HC Vy", () -> speeds.vyMetersPerSecond),
        new Measure("HC Omega", () -> speeds.omegaRadiansPerSecond)
//        new Measure("Odom. X", () -> odometryPose.getX()),
//        new Measure("Odom. Y", () -> odometryPose.getY()),
//        new Measure("Odom. Degrees", () -> odometryPose.getRotation().getDegrees())
    );
  }
}
