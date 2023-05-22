// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autos.pose;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
// import frc.robot.BreakerLib.auto.waypoint.BreakerPoseWaypointPath;
// import frc.robot.BreakerLib.auto.waypoint.BreakerSwervePoseWaypointPathFollower;
// import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
// import frc.robot.subsystems.Drive;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class SubPlace3 extends SequentialCommandGroup {
//     /** Creates a new SubPlaceLeaveThenBalance. */
//     public SubPlace3(Drive drive, BreakerPigeon2 imu) {

//         BreakerPoseWaypointPath wpp = new BreakerPoseWaypointPath(
//                 1.5,
//                 new Pose2d(1.856, 5, new Rotation2d()),
//                 new Pose2d(7.089, 4.588, Rotation2d.fromDegrees(180)),
//                 new Pose2d(1.856, 4.394, new Rotation2d()));

//         BreakerPoseWaypointPath wpp2 = new BreakerPoseWaypointPath(1.5,
//                 new Pose2d(5.395, 4.287, Rotation2d.fromDegrees(180)),
//                 new Pose2d(7.089, 3.363, Rotation2d.fromDegrees(180)),
//                 new Pose2d(5.395, 4.287, new Rotation2d()),
//                 new Pose2d(1.856, 4.394, new Rotation2d()),
//                 new Pose2d(1.856, 3.784, new Rotation2d()));

//         addCommands(
//                 new BreakerStartTrajectoryPath(drive, Drive.mirrorPathToAlliance(wpp).getWaypoints()[0]),
//                 new BreakerSwervePoseWaypointPathFollower(drive.autoConfig, true, Drive.mirrorPathToAlliance(wpp)),
//                 new WaitCommand(1),
//                 new BreakerSwervePoseWaypointPathFollower(drive.autoConfig, true, Drive.mirrorPathToAlliance(wpp2)),
//                 new WaitCommand(1));
//     }
// }
