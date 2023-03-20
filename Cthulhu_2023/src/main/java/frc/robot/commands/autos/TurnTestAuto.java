// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerPoseWaypointPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwervePoseWaypointPathFollower;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollowerConfig;
import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.subsystems.Drive;

/** Add your docs here. */
public class TurnTestAuto extends SequentialCommandGroup {

    public TurnTestAuto(Drive drive, BreakerPigeon2 imu) {
        ProfiledPIDController anglePID = new ProfiledPIDController(6.9, 0.0, 0.0,
                new TrapezoidProfile.Constraints(3, 3));
        PIDController drivePID = new PIDController(3, 0.0, 0.0);
        BreakerHolonomicDriveController driveController = new BreakerHolonomicDriveController(drivePID, anglePID);
        driveController.setTolerances(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(1.0)));

        BreakerPoseWaypointPath wpp = new BreakerPoseWaypointPath(
                1.5,
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(4.0, 0, Rotation2d.fromDegrees(180)),
                new Pose2d(0, 0, new Rotation2d())
                );

        BreakerSwerveWaypointFollowerConfig config = new BreakerSwerveWaypointFollowerConfig(drive, driveController);
        addCommands(
                new BreakerStartTrajectoryPath(drive,
                        Drive.mirrorPathToAlliance(wpp).getWaypoints()[0]),
                new BreakerSwervePoseWaypointPathFollower(config, true, wpp));
    }
}
