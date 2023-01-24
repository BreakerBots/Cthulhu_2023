// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.devices.vision.photon.BreakerFiducialPhotonTarget;

public class VisionTest extends SubsystemBase {
  /** Creates a new VisionTest. */
  private BreakerFiducialPhotonTarget tgt;
  public VisionTest() {
    tgt = new BreakerFiducialPhotonTarget(2, new Pose3d(0, 0,
    Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.toRadians(180))), Constants.Vision.AprilTag.APRILTAG_CAMERAS);
  }

  @Override
  public void periodic() {
    if (tgt.getAssignedTargetFound()) {
      System.out.println(tgt.getRobotPose3d());
    }
    
  }
}
