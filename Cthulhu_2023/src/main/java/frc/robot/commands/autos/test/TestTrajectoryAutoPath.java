// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autos.test;

// import edu.wpi.first.math.controller.HolonomicDriveController;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.BreakerLib.auto.trajectory.BreakerTrajectoryUtil;
// import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
// import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
// import frc.robot.BreakerLib.auto.trajectory.swerve.standard.BreakerSwerveAutoPathFollower;
// import frc.robot.BreakerLib.auto.trajectory.swerve.standard.BreakerSwerveAutoPathFollowerConfig;
// import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
// import frc.robot.commands.BalanceChargingStation;
// import frc.robot.subsystems.Drive;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TestTrajectoryAutoPath extends SequentialCommandGroup {

//   /** Creates a new TestAutoPath. */
//   public TestTrajectoryAutoPath(Drive drivetrain, BreakerPigeon2 imu) {

//     BreakerSwerveAutoPathFollowerConfig swerveFollowerConfig = new BreakerSwerveAutoPathFollowerConfig(drivetrain,
//         new HolonomicDriveController(new PIDController(2.0, 0.0, 0.1), new PIDController(2.0, 0.0, 0.1),
//         new ProfiledPIDController(0.000000001, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0))));

//     BreakerTrajectoryPath traj1 = new BreakerTrajectoryPath(TrajectoryGenerator.generateTrajectory(
//         new Pose2d(), // (0, 0). Start at left Tag 3 grid
//         BreakerTrajectoryUtil.toTranslationWaypointList(
//             new Translation2d(Units.feetToMeters(14), Units.feetToMeters(0)),
//             new Translation2d(Units.feetToMeters(1.5), Units.feetToMeters(0)),
//             new Translation2d(Units.feetToMeters(1.5), Units.feetToMeters(4))
//             ),
//             new Pose2d(new Translation2d(Units.feetToMeters(7), Units.feetToMeters(0)), Rotation2d.fromDegrees(0)),
//         new TrajectoryConfig(1.0, .75)), true);

//     addCommands(
//         new BreakerStartTrajectoryPath(drivetrain, new Pose2d()),
//         new BreakerSwerveAutoPathFollower(swerveFollowerConfig, traj1),
//         new BalanceChargingStation(drivetrain, imu));
//   }
// }