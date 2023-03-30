// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerPoseWaypointPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwervePoseWaypointPathFollower;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollower;
import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.commands.BalanceChargingStation;
import frc.robot.commands.MoveArmToState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.SebArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GatePlaceLeaveThenBalance extends SequentialCommandGroup {
  /** Creates a new TestWaypointAutoPath. */
  public GatePlaceLeaveThenBalance(Drive drive, BreakerPigeon2 imu, SebArm arm, RollerIntake intake) {

    BreakerPoseWaypointPath wpp = new BreakerPoseWaypointPath(
        2,
        new Pose2d(1.88, 1.085, new Rotation2d()),
        new Pose2d(5.724, 1.084, new Rotation2d()),
        new Pose2d(5.725, 2.726, new Rotation2d()),
        new Pose2d(3.802, 2.727, new Rotation2d()));
    addCommands(
        new BreakerStartTrajectoryPath(drive, Drive.mirrorPathToAlliance(wpp).getWaypoints()[0]),
        new MoveArmToState(arm, SebArm.State.PLACE_CUBE_MID),
        intake.ejectCmd(),
        new WaitCommand(0.25),
        intake.stopCmd(),
        new ParallelCommandGroup(
            new BreakerSwervePoseWaypointPathFollower(drive.autoConfig, true, Drive.mirrorPathToAlliance(wpp)),
            new MoveArmToState(arm, SebArm.State.STOW_CUBE)),
        new BalanceChargingStation(drive, imu));
  }
}
