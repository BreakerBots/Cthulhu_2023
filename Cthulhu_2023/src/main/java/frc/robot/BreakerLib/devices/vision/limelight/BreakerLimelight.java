// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightResults;

/** Add your docs here. */
public class BreakerLimelight {
    private final String limelightName;
    public BreakerLimelight(String limelightName) {
        this.limelightName = limelightName;
        
    }

    

    public Pair<Pose3d, Double> getFiducialRobotPoseAndLatancy() {
        double[] data = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
        return new Pair<Pose3d,Double>(new Pose3d(new Translation3d(data[0], data[1], data[2]), new Rotation3d(data[3], data[4], data[5])), data[6]);
    }

    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults(limelightName);
    }


}
