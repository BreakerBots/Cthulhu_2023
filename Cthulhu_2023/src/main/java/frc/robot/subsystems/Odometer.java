// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.AprilTag.APRILTAGS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveDriveFiducialVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionPoseFilter;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;

/** Add your docs here. */
public class Odometer extends BreakerSwerveDriveFiducialVisionPoseEstimator {
    private Alliance prevAlliance = DriverStation.getAlliance();
    public Odometer(BreakerSwerveDrive drivetrain) {
        super(
            drivetrain, 
            new BreakerVisionOdometer(
                new BreakerVisionPoseFilter(
                    0, 
                    0.5, 
                    APRILTAGS
                    )
                ), 
                null, 
                null, 
                null
            );
            CommandScheduler.getInstance().schedule(new RunCommand(this::checkAllianceTransposition));
    }

    /** Corrects for different expected angle zero points based on alliance, updates for FMS connection changes. */
    private void checkAllianceTransposition() {
        if (prevAlliance != DriverStation.getAlliance()) {
            setOdometryRotation(DriverStation.getAlliance() == Alliance.Blue ? getOdometryPoseMeters().getRotation() : getOdometryPoseMeters().getRotation().minus(Rotation2d.fromDegrees(180)));
        }
    }

    public java.awt.geom.Rectangle2D getRobotHitbox() {
        return Constants.RobotGeometry.ROBOT_GEOMETRY.getTranslatedHitbox(getOdometryPoseMeters());
    }

}
