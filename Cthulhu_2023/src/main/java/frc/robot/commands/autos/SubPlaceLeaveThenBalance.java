// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerPoseWaypointPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwervePoseWaypointPathFollower;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollowerConfig;
import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.commands.BalanceChargingStation;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SubPlaceLeaveThenBalance extends SequentialCommandGroup {
  /** Creates a new SubPlaceLeaveThenBalance. */
  public SubPlaceLeaveThenBalance(Drive drive, BreakerPigeon2 imu) {
    ProfiledPIDController anglePID = new ProfiledPIDController(0.000000001, 0.0, 0.0,
        new TrapezoidProfile.Constraints(0.0, 0.0));
    PIDController drivePID = new PIDController(2.0, 0, 0.0);
    BreakerHolonomicDriveController driveController = new BreakerHolonomicDriveController(drivePID, anglePID);
    driveController.setTolerances(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(180)));

    BreakerPoseWaypointPath wpp = new BreakerPoseWaypointPath(
        1.5,
        new Pose2d(1.856, 5, new Rotation2d()),
        new Pose2d(7.089, 4.588, Rotation2d.fromDegrees(180)),
        new Pose2d(1.856, 4.351, new Rotation2d()));

    BreakerPoseWaypointPath wpp2 = new BreakerPoseWaypointPath(1.5,
        new Pose2d(1.856, 3.148, new Rotation2d()),
        new Pose2d(3.919, 3.148, new Rotation2d()));

    BreakerSwerveWaypointFollowerConfig config = new BreakerSwerveWaypointFollowerConfig(drive, driveController);

    addCommands(new BreakerStartTrajectoryPath(drive, Drive.mirrorPathToAlliance(wpp).getWaypoints()[0]),
        new BreakerSwervePoseWaypointPathFollower(config, true, Drive.mirrorPathToAlliance(wpp)),
        new WaitCommand(1),
        new BreakerSwervePoseWaypointPathFollower(config, true, Drive.mirrorPathToAlliance(wpp2)),
        new BalanceChargingStation(drive, imu));
  }
}
