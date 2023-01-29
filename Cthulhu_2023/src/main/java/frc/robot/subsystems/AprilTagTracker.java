// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.photon.BreakerFiducialPhotonTarget;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;

public class AprilTagTracker extends SubsystemBase {

  private Translation3d cam1Translation = new Translation3d(Units.inchesToMeters(12.75), Units.inchesToMeters(2.25),
      Units.inchesToMeters(13.4));
  private Rotation3d cam1Rotation = new Rotation3d(); // 0 degrees in all angles
  private BreakerPhotonCamera cam1 = new BreakerPhotonCamera("April_Test_1",
      new Transform3d(cam1Translation, cam1Rotation));
  private BreakerFiducialPhotonTarget aprilTag3;
  private int i;

  public AprilTagTracker() {
    Rotation3d aprilTag3Rotation = new Rotation3d(0, 0, Math.toRadians(0));
    Pose3d aprilTag3Pose = new Pose3d(0, 0, Units.inchesToMeters(18.22), aprilTag3Rotation);
    aprilTag3 = new BreakerFiducialPhotonTarget(3, aprilTag3Pose, cam1);
  }

  public boolean tgtFound() {
    return aprilTag3.getAssignedTargetFound();
  }

  public Pose2d getRobotPose() {
    return aprilTag3.getRobotPose();
  }

  @Override
  public void periodic() {
    if (aprilTag3.getAssignedTargetFound() && ((i++)%25==0)) {
      //System.out.println(aprilTag3.getRobotPose() + " DATA AGE: " + aprilTag3.getTargetDataAge());
    }

  }
}
