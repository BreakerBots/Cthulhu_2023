// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gamepiece;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Objects;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;

/** Upright cone, toppeld cone, or cube, each given a predefined height. */
enum TrackedGamePieceType {
  CONE_UPRIGHT(Units.inchesToMeters(12.8125)),
  CONE_TOPPLED(Units.inchesToMeters(8.375)),
  CUBE(Units.inchesToMeters(9.5));

  private double heightMeters;

  private TrackedGamePieceType(double heightMeters) {
    this.heightMeters = heightMeters;
  }

  public double getHeightMeters() {
    return heightMeters;
  }

}

public class GamePieceTracker extends SubsystemBase {

  private BreakerPhotonCamera /* coneCam, */ cubeCam;
  private double uprightConeBoundRatioThreshold = 1.2; // H:W threshold between upright and toppled cones.
  private ArrayList<TrackedGamePiece> trackedGamePieces;

  /** Creates a new GamePieceTracker. */
  public GamePieceTracker() {
    // coneCam = new BreakerPhotonCamera("CONE_CAM_1", new Transform3d());
    // cubeCam = new BreakerPhotonCamera("CUBE_CAM_1", new Transform3d());
    cubeCam = new BreakerPhotonCamera("April_Test_1",
        new Transform3d(new Translation3d(Units.inchesToMeters(12.75), Units.inchesToMeters(2.25),
            Units.inchesToMeters(13.0)), new Rotation3d()));
    trackedGamePieces = new ArrayList<>();
  }

  private boolean isConeUpright(double coneBoxWidth, double coneBoxHeight) {
    return coneBoxHeight / coneBoxWidth >= uprightConeBoundRatioThreshold;
  }

  /**
   * @param target Target to be given a bounding box.
   * 
   * @return Array of 2 with width in slot 0 and height in slot 1
   */
  private double[] makeBoundingBox(PhotonTrackedTarget target) {
    var corners = target.getMinAreaRectCorners();
    TargetCorner firstCorner = corners.remove(0);
    TargetCorner opposingCorner = firstCorner;

    for (TargetCorner tc : corners) {
      if (tc.x != firstCorner.x && tc.y != firstCorner.y) {
        opposingCorner = tc;
      }
    }

    double width = Math.abs(opposingCorner.x - firstCorner.x);
    double height = Math.abs(opposingCorner.y - firstCorner.y);

    return new double[] { width, height };
  }

  /** @return If any targets have been successfully found. */
  public boolean hasTargets() {
    return !trackedGamePieces.isEmpty();
  }

  /** @return Best overall target. */
  public TrackedGamePiece getBestTrackedGamePiece() {
    return trackedGamePieces.get(0);
  }

  /** @return Clone of tracked game pieces list. */
  public ArrayList<TrackedGamePiece> getTrackedGamePieces() {
    return new ArrayList<>(trackedGamePieces);
  }

  public void generateGamePieceList() {
    trackedGamePieces.clear();
    // Checks cones
    // if (coneCam.hasTargets()) {
    //   for (PhotonTrackedTarget target : cubeCam.getAllRawTrackedTargets()) {
    //     if (!Objects.isNull(target)) {
    //       double[] boundingBox = makeBoundingBox(target);
    //       TrackedGamePieceType coneType = isConeUpright(boundingBox[0], boundingBox[1]) ? TrackedGamePieceType.CONE_UPRIGHT
    //           : TrackedGamePieceType.CONE_TOPPLED;
    //       TrackedGamePiece cone = new TrackedGamePiece(TrackedGamePieceType.CONE_UPRIGHT,
    //           coneCam.get3dCamPositionRelativeToRobot(), target);
    //       trackedGamePieces.add(cone);
    //     }
    //   }
    // }
    // Checks cubes
    if (cubeCam.hasTargets()) {
      for (PhotonTrackedTarget target : cubeCam.getAllRawTrackedTargets()) {
        if (!Objects.isNull(target)) {
          TrackedGamePiece cube = new TrackedGamePiece(TrackedGamePieceType.CUBE,
              cubeCam.get3dCamPositionRelativeToRobot(), target);
          trackedGamePieces.add(cube);
        }
      }
    }
    // Sorts the non-empty list
    if (!trackedGamePieces.isEmpty()) {
      Collections.sort(trackedGamePieces);
      // System.out.println(trackedGamePieces);
    }
  }

  @Override
  public void periodic() {
    generateGamePieceList();
  }

  public static class TrackedGamePiece implements Comparable<TrackedGamePiece> {
    private final double CAMERA_FOV_PITCH_RAD = 1.1;

    private Transform3d cameraTransform;
    private TrackedGamePieceType type;
    private PhotonTrackedTarget target;

    /**
     * Makes a trackable game piece instance which can be compared.
     * 
     * @param type            Target's type
     * @param cameraTransform Camera transform relative to robot's bottom center.
     * @param target          PhotonTrackedTarget from a camera to perform
     *                        operations on.
     */
    private TrackedGamePiece(TrackedGamePieceType type, Transform3d cameraTransform, PhotonTrackedTarget target) {
      this.target = target;
      this.type = type;
      this.cameraTransform = cameraTransform;
    }

    /** @return 2d translation from robot to target. */
    public Translation2d getRobotToTargetTranslation() {
      return PhotonUtils.estimateCameraToTargetTranslation(getDistance(), Rotation2d.fromDegrees(target.getYaw()))
          .plus(cameraTransform.getTranslation().toTranslation2d());
    }

    /**
     * Calculates distance based upon the relative heights and pitches of the camera
     * and target.
     * 
     * @return Distance magnitude from robot to target in meters.
     */
    public double getDistance() {
      // TODO Why do we divide the type's height by 2?
      return PhotonUtils.calculateDistanceToTargetMeters(cameraTransform.getZ(), type.getHeightMeters() / 2.0,
          cameraTransform.getRotation().getY(), Math.toRadians(target.getPitch()));
    }

    public PhotonTrackedTarget getTarget() {
      return target;
    }

    public TrackedGamePieceType getType() {
      return type;
    }

    @Override
    public String toString() {
      // TODO Auto-generated method stub
      return "Dist: " + getDistance();
    }

    @Override
    public int compareTo(TrackedGamePiece arg0) {

      double otherDist = Math.hypot(arg0.target.getYaw(),
          Math.abs(arg0.target.getPitch() + (CAMERA_FOV_PITCH_RAD / 2)));
      double dist = Math.hypot(target.getYaw(), Math.abs(target.getPitch() + (CAMERA_FOV_PITCH_RAD / 2)));
      // double otherDist = arg0.getDistance();// + Math.abs(arg0.target.getYaw());
      // double dist = getDistance();// + Math.abs(target.getYaw());
      if (dist > otherDist) {
        return 1;
      } else if (dist < otherDist) {
        return -1;
      } else {
        return 0;
      }
    }
  }

}
