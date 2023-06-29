// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.devices.vision.photon.BreakerVision;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionPoseFilterOdometer;

public class Vision extends SubsystemBase implements BreakerGenericVisionOdometer {
  /** Creates a new Vision. */
  private BreakerPhotonCamera frontCam, leftCam, rightCam, backCam;
  private BreakerVision vision;
  public Vision() {
    frontCam = new BreakerPhotonCamera(getName(), null);
    leftCam  = new BreakerPhotonCamera(getName(), null);
    rightCam = new BreakerPhotonCamera(getName(), null);
    backCam = new BreakerPhotonCamera(getName(), null);
    vision = new BreakerVision(getDataTimestamp(), getDataTimestamp(), getDataTimestamp(), getDataTimestamp(), new BreakerPhotonCamera[]{frontCam, leftCam, rightCam, backCam}, null);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void setOdometryPosition(Pose2d newPose) {
    vision.setOdometryPosition(newPose);
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return vision.getOdometryPoseMeters();
  }

  @Override
  public BreakerMovementState2d getMovementState() {
    return vision.getMovementState();
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return vision.getRobotRelativeChassisSpeeds();
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return vision.getFieldRelativeChassisSpeeds();
  }

  @Override
  public double getDataTimestamp() {
    return vision.getBaseVisionOdometer().getDataTimestamp();
  }

  @Override
  public boolean isAnyTargetVisable() {
    return vision.getBaseVisionOdometer().isAnyTargetVisable();
  }

  public static class VisionConstants {
    public static final String FRONT_CAMERA_NAME = "frontCam";
    public static final String LEFT_CAMERA_NAME = "leftCam";
    public static final String RIGHT_CAMERA_NAME = "rightCam";
    public static final String BACK_CAMERA_NAME = "backCam";

    public static final Transform3d FRONT_CAMERA_POSE = new Transform3d();
    public static final Transform3d LEFT_CAMERA_POSE = new Transform3d();
    public static final Transform3d RIGHT_CAMERA_POSE = new Transform3d();
    public static final Transform3d BACK_CAMERA_POSE = new Transform3d();
  }
}
