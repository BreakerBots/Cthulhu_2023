// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.Rect2d;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.photon.BreakerFiducialPhotonTarget;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonTarget2d;

enum GamePieceType {
  CONE_UPRIGHT,
  CONE_TOPPLED,
  CUBE,
  NONE
}

public class GamePieceTracker extends SubsystemBase {
  private BreakerPhotonCamera coneCam, cubeCam;
  private PhotonTrackedTarget bestCone, bestCube;
  private double datatTimestamp;
  private GamePieceType currentPieceType;

  
  /** Creates a new GamePieceVision. */
  public GamePieceTracker() {
    coneCam = new BreakerPhotonCamera(DUMMY, DUMMY);
    cubeCam = new BreakerPhotonCamera(DUMMY, DUMMY);
  }

  public Pair<Double, Double> makeBoundingBox(PhotonTrackedTarget target) {
    var corners = target.getDetectedCorners();
    TargetCorner firstCorner = corners.remove(0);
    for (TargetCorner tc: corners) {
      if (tc.x != firstCorner.x && tc.y != firstCorner.y) {
        return new Pair();
      }
    }
  }

  @Override
  public void periodic() {
    if (coneCam.hasTargets()) {
      // Assuming cones are placed into rectangular bounding boxes
      bestCone = coneCam.getBestTarget();
    }
    else if (cubeCam.hasTargets()) {
      bestCube = cubeCam.getBestTarget();
      currentPieceType = GamePieceType.CUBE;
    }
  }
}
