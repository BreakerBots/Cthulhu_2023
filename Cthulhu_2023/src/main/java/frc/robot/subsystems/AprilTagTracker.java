// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

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
  private BreakerFiducialPhotonTarget aprilTag1;
  private BreakerFiducialPhotonTarget aprilTag2;
  private BreakerFiducialPhotonTarget aprilTag3;
  private BreakerFiducialPhotonTarget aprilTag6;
  private BreakerFiducialPhotonTarget aprilTag7;
  private BreakerFiducialPhotonTarget aprilTag8;
  //private BreakerVisionPoseFilter poseFilter;
  private int i;

  private ArrayList<BreakerFiducialPhotonTarget> aprilList = new ArrayList<>();
  
  public AprilTagTracker() {

    Pose3d aprilTag3Pose = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19),
                                 Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.toRadians(180)));
    Pose3d aprilTag2Pose = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19),
                                 Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.toRadians(180)));
    Pose3d aprilTag1Pose = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19),
                                 Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.toRadians(180)));
    
    Pose3d aprilTag6Pose = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19),
                                 Units.inchesToMeters(18.22), new Rotation3d(0, 0, 0));
    Pose3d aprilTag7Pose = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19),
                                 Units.inchesToMeters(18.22), new Rotation3d(0, 0, 0));
    Pose3d aprilTag8Pose = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19),
                                 Units.inchesToMeters(18.22), new Rotation3d(0, 0, 0));
    
    aprilTag8 = new BreakerFiducialPhotonTarget(8, aprilTag8Pose, cam1);
    aprilTag7 = new BreakerFiducialPhotonTarget(7, aprilTag7Pose, cam1);
    aprilTag6 = new BreakerFiducialPhotonTarget(6, aprilTag6Pose, cam1);                            
    aprilTag3 = new BreakerFiducialPhotonTarget(3, aprilTag3Pose, cam1);
    aprilTag2 = new BreakerFiducialPhotonTarget(2, aprilTag2Pose, cam1);
    aprilTag1 = new BreakerFiducialPhotonTarget(1, aprilTag1Pose, cam1);
    aprilList.add(aprilTag1);
    aprilList.add(aprilTag2);
    aprilList.add(aprilTag3);
    aprilList.add(aprilTag6);
    aprilList.add(aprilTag7);
    aprilList.add(aprilTag8);

    //poseFilter = new BreakerVisionPoseFilter(2.0, 0.6, aprilTag1, aprilTag2, aprilTag3);
  }

  public boolean tgtFound() {
   for (BreakerFiducialPhotonTarget tgt: aprilList) {
    if (tgt.getAssignedTargetFound()) {
      return true;
    }
   }
   return false;
  }

  public boolean camConnected() {
    return !cam1.hasFault();
  }

  public Pose2d getRobotPose() {
    double lowestUncertainty = 1.0;
    int indexOfBest = 0;
    for (int i = 0; i < aprilList.size(); i++) {
      var aprilTag = aprilList.get(i);
      if (aprilTag.getAssignedTargetFound()) {
        double uncertainty = aprilTag.getPoseAmbiguity();
        if (uncertainty < lowestUncertainty) {
          lowestUncertainty = uncertainty;
          indexOfBest = i;
        }
      }
    }
    return aprilList.get(indexOfBest).getRobotPose();
  }

  @Override
  public void periodic() {
    if (aprilTag2.getAssignedTargetFound() && ((i++)%25==0)) {
      //System.out.println(aprilTag3.getRobotPose() + " DATA AGE: " + aprilTag3.getTargetDataAge());
    }

  }
}
