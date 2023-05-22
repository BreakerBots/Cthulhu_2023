// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autos.test;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
// import frc.robot.BreakerLib.auto.waypoint.BreakerPoseWaypointPath;
// import frc.robot.BreakerLib.auto.waypoint.BreakerSwervePoseWaypointPathFollower;
// import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
// import frc.robot.subsystems.Drive;

// /** Add your docs here. */
// public class TurnTestAuto extends SequentialCommandGroup {

//     public TurnTestAuto(Drive drive, BreakerPigeon2 imu) {

//         BreakerPoseWaypointPath wpp = new BreakerPoseWaypointPath(
//                 1.5,
//                 new Pose2d(0, 0, new Rotation2d()),
//                 new Pose2d(4.0, 0, Rotation2d.fromDegrees(180)),
//                 new Pose2d(0, 0, new Rotation2d())
//                 );

//         addCommands(
//                 new BreakerStartTrajectoryPath(drive,
//                         Drive.mirrorPathToAlliance(wpp).getWaypoints()[0]),
//                 new BreakerSwervePoseWaypointPathFollower(drive.autoConfig, true, wpp));
//     }
// }
