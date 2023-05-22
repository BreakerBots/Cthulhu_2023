// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autos;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
// import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollower;
// import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
// import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
// import frc.robot.commands.BalanceChargingStation;
// import frc.robot.subsystems.AprilTagTracker;
// import frc.robot.subsystems.Drive;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class Pickup1_Place2_Balance_6_3 extends SequentialCommandGroup {
//   /** Creates a new ApriltagTestPath. */
//   public Pickup1_Place2_Balance_6_3(Drive drive, AprilTagTracker att, BreakerPigeon2 imu) {

//     BreakerWaypointPath wpp = new BreakerWaypointPath(
//         1.0, 
//         new Translation2d(5.7, 4.0)
//         );
//     BreakerWaypointPath wpp2 = new BreakerWaypointPath(
//           1.0, 
//           new Translation2d(1.4, 4.0)//0.91
//           );
//     BreakerWaypointPath wpp3 = new BreakerWaypointPath(
//           2.0, 
//           new Translation2d(2.5, 2.74),
//           new Translation2d(3.6, 2.74)
//           );
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//         new WaitUntilCommand(att::tgtFound),
//         new WaitCommand(1.0),
//         new BreakerStartTrajectoryPath(drive, att.camConnected() ? att.getRobotPose() : new Pose2d()),
//         new WaitCommand(1.0),
//         new BreakerSwerveWaypointFollower(drive.autoConfig, true, Drive.mirrorPathToAlliance(wpp)),
//         new WaitCommand(1.0),
//         new BreakerSwerveWaypointFollower(drive.autoConfig, true,  Drive.mirrorPathToAlliance(wpp2)),
//         new WaitCommand(1.0),
//         new BreakerSwerveWaypointFollower(drive.autoConfig, true,  Drive.mirrorPathToAlliance(wpp3)),
//         new BalanceChargingStation(drive, imu)
//     );
//   }
// }
