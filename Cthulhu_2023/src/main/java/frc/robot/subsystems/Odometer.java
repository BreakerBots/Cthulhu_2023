// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveDriveFiducialVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveDrivePoseEstimator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionPoseFilter;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import static frc.robot.Constants.Vision.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Odometer extends BreakerSwerveDriveFiducialVisionPoseEstimator {
    public Odometer(BreakerSwerveDrive drivetrain) {
        super(
            drivetrain, 
            new BreakerVisionOdometer(
                new BreakerVisionPoseFilter(
                    0, 
                    0, 
                    APRILTAGS
                    )
                ), 
                null, 
                null, 
                null
            );
    }

}
