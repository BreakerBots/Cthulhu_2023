// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autos.test;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
// import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollower;
// import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollowerConfig;
// import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
// import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
// import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
// import frc.robot.commands.BalanceChargingStation;
// import frc.robot.subsystems.Drive;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TestWaypointAutoPath extends SequentialCommandGroup {
//   /** Creates a new TestWaypointAutoPath. */
//   public TestWaypointAutoPath(Drive drive, BreakerPigeon2 imu) {
//     ProfiledPIDController anglePID = new ProfiledPIDController(0.000000001, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
//     PIDController drivePID = new PIDController(2.0, 0, 0.0);
//     BreakerHolonomicDriveController driveController = new BreakerHolonomicDriveController(drivePID, anglePID);
//     driveController.setTolerances(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(180)));
    
//       BreakerWaypointPath wpp = new BreakerWaypointPath(
//         0.5, 
//         new Translation2d(Units.feetToMeters(14), Units.feetToMeters(0))
//         );

//         BreakerWaypointPath wpp2 = new BreakerWaypointPath(
//           0.25,
//           new Translation2d(Units.feetToMeters(0.75), Units.feetToMeters(0))
//         );

//         BreakerWaypointPath wpp3 = new BreakerWaypointPath(
//           0.5, 
//           new Translation2d(Units.feetToMeters(0.75), Units.feetToMeters(6.5))
//         );

//         BreakerWaypointPath wpp4 = new BreakerWaypointPath(
//           1.0,
//           new Translation2d(Units.feetToMeters(5.5), Units.feetToMeters(6.5))
//         );

//     BreakerSwerveWaypointFollowerConfig config = new BreakerSwerveWaypointFollowerConfig(drive, driveController);
//     addCommands(
//         new BreakerStartTrajectoryPath(drive, new Pose2d()),
//         new BreakerSwerveWaypointFollower(config, true, wpp),
//         new WaitCommand(1.5),
//         new BreakerSwerveWaypointFollower(config, true, wpp2),
//         new WaitCommand(1.5),
//         new BreakerSwerveWaypointFollower(config, true, wpp3),
//         new BreakerSwerveWaypointFollower(config, true, wpp4),
//         new BalanceChargingStation(drive, imu)
//         );
//   }
// }
