// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;

import org.opencv.core.Rect2d;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.photon.BreakerFiducialPhotonTarget;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonTarget2d;

enum GamePieceType {
  CONE_UPRIGHT(Units.inchesToMeters(12.8125)),
  CONE_TOPPLED(Units.inchesToMeters(8.375)),
  CUBE(Units.inchesToMeters(9.5));

  private double heightMeters;

  private GamePieceType(double hightMeters) {
    this.heightMeters = hightMeters;
  }

  public double getHeightMeters() {
    return heightMeters;
  }

}

class TrackedGamePiece implements Comparable<TrackedGamePiece> {
  private final double CAMERA_FOV_PITCH_RAD = 0;

  private Transform3d cameraTransform;
  private GamePieceType type;
  private PhotonTrackedTarget target;

  public TrackedGamePiece(GamePieceType type, Transform3d cameraTransform, PhotonTrackedTarget target) {
    this.target = target;
    this.type = type;
    this.cameraTransform = cameraTransform;
  }

  public Translation2d getRobotToTargetTranslation() {
    return PhotonUtils.estimateCameraToTargetTranslation(getDistance(), Rotation2d.fromDegrees(target.getYaw()))
        .plus(cameraTransform.getTranslation().toTranslation2d());
  }

  public double getDistance() {
    return PhotonUtils.calculateDistanceToTargetMeters(cameraTransform.getZ(), type.getHeightMeters(),
        cameraTransform.getRotation().getY(), Math.toRadians(target.getPitch()));
  }

  public PhotonTrackedTarget getTarget() {
    return target;
  }

  public GamePieceType getType() {
    return type;
  }

  @Override
  public int compareTo(TrackedGamePiece arg0) {

    double otherDist = Math.hypot(arg0.target.getYaw(),
        Math.abs(arg0.target.getPitch() + (CAMERA_FOV_PITCH_RAD / 2)));
    double dist = Math.hypot(target.getYaw(), Math.abs(target.getPitch() + (CAMERA_FOV_PITCH_RAD / 2)));
    if (dist < otherDist) {
      return 1;
    } else if (dist > otherDist) {
      return -1;
    } else {
      return 0;
    }
  }
}

class VisionBoundingBox {
  private double width, height;

  public VisionBoundingBox(double width, double height) {
    this.width = width;
    this.height = height;
  }

  public double getHeight() {
    return height;
  }

  public double getWidth() {
    return width;
  }

}

public class GamePieceTracker extends SubsystemBase {

  private BreakerPhotonCamera coneCam, cubeCam;
  private double uprightConeBoundRatioThreshold = 1.2;
  private ArrayList<TrackedGamePiece> trackedGamePieces;

  /** Creates a new GamePieceTracker. */
  public GamePieceTracker() {
    coneCam = new BreakerPhotonCamera("CONE_CAM_1", new Transform3d());
    cubeCam = new BreakerPhotonCamera("CUBE_CAM_1", new Transform3d());
    trackedGamePieces = new ArrayList<>();
  }

  private boolean isConeUpright(VisionBoundingBox coneBound) {
    return coneBound.getHeight() / coneBound.getWidth() >= uprightConeBoundRatioThreshold;
  }

  private VisionBoundingBox makeBoundingBox(PhotonTrackedTarget target) {
    var corners = target.getMinAreaRectCorners();
    TargetCorner firstCorner = corners.remove(0);
    TargetCorner opposingCorner = firstCorner;

    for (TargetCorner tc : corners) {
      if (tc.x != firstCorner.x && tc.y != firstCorner.y) {
        opposingCorner = tc;
      }
    }

    return new VisionBoundingBox(Math.abs(opposingCorner.x - firstCorner.x),
        Math.abs(opposingCorner.y - firstCorner.y));
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
      for (PhotonTrackedTarget target : coneCam.getAllRawTrackedTargets()) {
        trackedGamePieces.add(new TrackedGamePiece(
            isConeUpright(makeBoundingBox(target)) ? GamePieceType.CONE_UPRIGHT : GamePieceType.CONE_TOPPLED,
            coneCam.get3dCamPositionRelativeToRobot(), target));
      }
    }
    if (cubeCam.hasTargets()) {
      for (PhotonTrackedTarget target : cubeCam.getAllRawTrackedTargets()) {
        trackedGamePieces
            .add(new TrackedGamePiece(GamePieceType.CUBE, cubeCam.get3dCamPositionRelativeToRobot(), target));
      }
    }
    if (!trackedGamePieces.isEmpty()) {
      Collections.sort(trackedGamePieces);
    }
  }

}
