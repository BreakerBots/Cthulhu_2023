// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.limelight;

import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightResults;

/** Add your docs here. */
public class BreakerLimelight {
    private final String limelightName;
    public BreakerLimelight(String limelightName) {
        this.limelightName = limelightName;
        
    }

    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults(limelightName);
    }


}
