// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autos;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
// import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollower;
// import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
// import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
// import frc.robot.commands.BalanceChargingStation;
// import frc.robot.subsystems.Drive;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class InNOut extends SequentialCommandGroup {

//   public InNOut(Drive drive, BreakerPigeon2 imu) {

//     BreakerWaypointPath wpp = new BreakerWaypointPath(
//         1.5,
//         new Translation2d(1.88, 0.453),
//         new Translation2d(2.88, 0.453),
//         new Translation2d(1.88, 0.453),
//         new Translation2d(5.895, 0.453));

//     addCommands(
//         new BreakerStartTrajectoryPath(drive, new Pose2d(Drive.mirrorPathToAlliance(wpp).getWaypoints()[0],
//             DriverStation.getAlliance() == Alliance.Red ? Rotation2d.fromDegrees(180) : new Rotation2d())),
//         new BreakerSwerveWaypointFollower(drive.autoConfig, true, Drive.mirrorPathToAlliance(wpp)),
//         new BalanceChargingStation(drive, imu));
//   }
// }
