// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerSwerveWaypointFollower extends CommandBase {
  private BreakerSwerveWaypointFollowerConfig config;
  private final Timer timer = new Timer();
  private HolonomicDriveController driveController;
  private BreakerWaypointPath waypointPath;
  private Supplier<Rotation2d> rotationSupplier;
  private Translation2d prevWp;
  private ArrayList<Translation2d> waypoints;
  private double totalDistance;

  /**
   * Create a BreakerSwerveWaypointFollower with no rotation supplier.
   * 
   * @param config Config for the follower.
   * @param waypointPath Path to follow.
   */
  public BreakerSwerveWaypointFollower(BreakerSwerveWaypointFollowerConfig config, BreakerWaypointPath waypointPath) {
    addRequirements(config.getDrivetrain());
    waypoints = new ArrayList<>();
    for (Translation2d wp : waypointPath.getWaypoints()) {
      waypoints.add(wp);
    }
    prevWp = config.getOdometer().getOdometryPoseMeters().getTranslation();
    totalDistance = waypointPath.getTotalPathDistance() + waypoints.get(0).getDistance(prevWp);
    this.config = config;
    this.waypointPath = waypointPath;
    rotationSupplier = () -> (BreakerMath.getPointAngleRelativeToOtherPoint(prevWp, waypoints.get(0)));
    driveController = config.getDriveController();
  }

  /**
   * Create a BreakerSwerveWaypointFollower with rotation supplier.
   * 
   * @param config Config for the follower.
   * @param waypointPath Path to follow.
   * @param rotationSupplier Supplier of swerve rotation. Useful for CV target tracking et al.
   */
  public BreakerSwerveWaypointFollower(BreakerSwerveWaypointFollowerConfig config, BreakerWaypointPath waypointPath,
      Supplier<Rotation2d> rotationSupplier) {
    addRequirements(config.getDrivetrain());
    waypoints = new ArrayList<>();
    for (Translation2d wp : waypointPath.getWaypoints()) {
      waypoints.add(wp);
    }

    totalDistance = waypointPath.getTotalPathDistance();
    this.config = config;
    this.waypointPath = waypointPath;
    this.rotationSupplier = rotationSupplier;
    driveController = config.getDriveController();
  }

  /** Sets follower to follow new waypoint path.
   * 
   * @param newWaypointPath Path to follow.
   */
  public void setWaypointPath(BreakerWaypointPath newWaypointPath) {
    waypointPath = newWaypointPath;
    waypoints.clear();
    for (Translation2d wp : waypointPath.getWaypoints()) {
      waypoints.add(wp);
    }
    totalDistance = waypointPath.getTotalPathDistance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    BreakerLog.logBreakerLibEvent("A new BreakerSwerveWaypointFollower instance has started");
    prevWp = config.getOdometer().getOdometryPoseMeters().getTranslation();
    totalDistance += +waypoints.get(0).getDistance(prevWp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Current values
    Pose2d curPose = config.getOdometer().getOdometryPoseMeters();
    BreakerVector2 curVelVec = config.getOdometer().getMovementState().getDerivativefromIndex(0).getLinearForces(); // 2d velocity vector
    double curVel = curVelVec.getMagnitude() * (curVelVec.getVectorRotation().getDegrees() >= 0 ? 1 : -1); // Current velocity in m/s
    TrapezoidProfile.State curState = new TrapezoidProfile.State(totalDistance - getTotalRemainingDistance(curPose),
        curVel);

    // Next chassis speeds are generated from updated trapezoid profile
    TrapezoidProfile profile = new TrapezoidProfile(waypointPath.getConstraints(),
        new TrapezoidProfile.State(totalDistance, 0), curState);
    Rotation2d targetRot = rotationSupplier.get();
    double profiledVel = profile.calculate(0.20).velocity;
    ChassisSpeeds targetSpeeds = driveController.calculate(curPose, new Pose2d(waypoints.get(0), targetRot),
        profiledVel, targetRot);
    targetSpeeds = new ChassisSpeeds(
        MathUtil.clamp(targetSpeeds.vxMetersPerSecond, -waypointPath.getConstraints().maxVelocity,
            waypointPath.getConstraints().maxVelocity),
        MathUtil.clamp(targetSpeeds.vyMetersPerSecond, -waypointPath.getConstraints().maxVelocity,
            waypointPath.getConstraints().maxVelocity),
        targetSpeeds.omegaRadiansPerSecond);

    // Robot is moved
    config.getDrivetrain().move(
        ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, config.getOdometer().getOdometryPoseMeters().getRotation()),
        false);
    System.out.println(profile.calculate(0.20).velocity);
    if (driveController.atReference()) {
      prevWp = waypoints.remove(0);
    }
  }

  /**
   * @return the internal list that represnets the queue of un-passed waypoints,
   *         can be modified
   */
  public ArrayList<Translation2d> getWaypoints() {
    return waypoints;
  }

  private double getDistanceToWaypoint(Pose2d curPose, Translation2d nextWp) {
    return curPose.getTranslation().getDistance(nextWp);
  }

  private double getTotalRemainingDistance(Pose2d curPose) {
    double totalDist = getDistanceToWaypoint(curPose, waypoints.get(0));
    for (int i = 1; i < waypoints.size(); i++) {
      totalDist += waypoints.get(i - 1).getDistance(waypoints.get(i));
    }
    return totalDist;
  }

  /** @return Elapsed path time in seconds. */
  public double getElapsedTimeSeconds() {
    return timer.get();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BreakerLog.logBreakerLibEvent("A BreakerSwerveWaypointFollower instance has ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return waypoints.isEmpty();
  }

}
