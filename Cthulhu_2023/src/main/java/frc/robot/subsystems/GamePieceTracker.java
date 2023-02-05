// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;

import org.opencv.core.Rect2d;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
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
  private double uprightConeBoundRatioThreshold = 1.2;
  private ArrayList<TrackedGamePiece> trackedGamePieces;

  
  /** Creates a new GamePieceVision. */
  public GamePieceTracker() {
    coneCam = new BreakerPhotonCamera("CONE_CAM_1", new Transform3d());
    cubeCam = new BreakerPhotonCamera("CUBE_CAM_1", new Transform3d());
    trackedGamePieces = new ArrayList<>();
  }

  private boolean isConeUpright(VisionBoundingBox coneBound) {
    return coneBound.getHight() / coneBound.getWidth() >= uprightConeBoundRatioThreshold;
  } 

  private VisionBoundingBox makeBoundingBox(PhotonTrackedTarget target) {
    var corners = target.getMinAreaRectCorners();
    TargetCorner firstCorner = corners.remove(0);
    TargetCorner opposingCorner = firstCorner;
    for (TargetCorner tc: corners) {
      if (tc.x != firstCorner.x && tc.y != firstCorner.y) {
        opposingCorner = tc;
      }
    }
    return new VisionBoundingBox(Math.abs(opposingCorner.x - firstCorner.x), Math.abs(opposingCorner.y - firstCorner.y));
  }

  public boolean hasTargets() {
    return !trackedGamePieces.isEmpty();
  }

  public TrackedGamePiece getBestTrackedGamePiece() {
    return trackedGamePieces.get(0);
  }

  public ArrayList<TrackedGamePiece> getTrackedGamePieces() {
    return new ArrayList<>(trackedGamePieces);
  }

 

  @Override
  public void periodic() {
    trackedGamePieces.clear();
    if (coneCam.hasTargets()) {
      for (PhotonTrackedTarget target: coneCam.getAllRawTrackedTargets()) {
        trackedGamePieces.add(new TrackedGamePiece(isConeUpright(makeBoundingBox(target)) ?  GamePieceType.CONE_UPRIGHT : GamePieceType.CONE_TOPPLED, target));
      }
    }
    if (cubeCam.hasTargets()) {
      for (PhotonTrackedTarget target: cubeCam.getAllRawTrackedTargets()) {
        trackedGamePieces.add(new TrackedGamePiece(GamePieceType.CUBE, target));
      }
    }
    if (!trackedGamePieces.isEmpty()) {
      Collections.sort(trackedGamePieces);
    }
  }

  public static class TrackedGamePiece implements Comparable<TrackedGamePiece> {
    private final double CAMERA_FOV_PITCH = 0;
    private GamePieceType type;
    private PhotonTrackedTarget target;
    public TrackedGamePiece(GamePieceType type, PhotonTrackedTarget target) {
      this.target = target;
      this.type = type;
    }

    public PhotonTrackedTarget getTarget() {
        return target;
    }

    public GamePieceType getType() {
        return type;
    }

    @Override
    public int compareTo(TrackedGamePiece arg0) {

      double otherDist = Math.hypot(arg0.target.getYaw(), Math.abs(arg0.target.getPitch() + (CAMERA_FOV_PITCH/2)));
      double dist = Math.hypot(target.getYaw(), Math.abs(target.getPitch() + (CAMERA_FOV_PITCH/2)));
      if (dist < otherDist) {
        return 1;
      } else if (dist > otherDist) {
        return -1;
      } else {
        return 0;
      }
    }
  }

  public static class VisionBoundingBox {
    private double width, hight;
    public VisionBoundingBox(double width, double hight) {
      this.width = width;
      this.hight = hight;
    }


    public double getHight() {
        return hight;
    }

    public double getWidth() {
        return width;
    }

  } 
}
