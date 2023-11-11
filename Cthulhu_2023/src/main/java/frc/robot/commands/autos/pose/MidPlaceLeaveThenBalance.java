// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerPoseWaypointPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwervePoseWaypointPathFollower;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollower;
import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerLegacyPigeon2;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.commands.BalanceChargingStation;
import frc.robot.commands.MoveArmToState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.SebArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidPlaceLeaveThenBalance extends SequentialCommandGroup {
  /** Creates a new TestWaypointAutoPath. */
  public MidPlaceLeaveThenBalance(Drive drive, BreakerLegacyPigeon2 imuSys, SebArm arm, RollerIntake intake) {

    BreakerPoseWaypointPath wpp = new BreakerPoseWaypointPath(
        1.5,
        new Pose2d(2.062, 2.721, new Rotation2d()),
        new Pose2d(7.41, 2.722, new Rotation2d()),
        new Pose2d(4.802, 2.721, new Rotation2d()));

    addCommands(

        new BreakerStartTrajectoryPath(drive, Drive.mirrorPathToAlliance(wpp).getWaypoints()[0]),
        new MoveArmToState(arm, SebArm.State.PLACE_CUBE_MID),
        intake.ejectCmd(),
        new WaitCommand(0.25),
        intake.stopCmd(),
        new MoveArmToState(arm, SebArm.State.STOW_CUBE),
        // new SequentialCommandGroup(
        new BreakerSwervePoseWaypointPathFollower(drive.autoConfig, true, Drive.mirrorPathToAlliance(wpp)),
        new BalanceChargingStation(drive, imuSys),
        new InstantCommand(() -> drive.move(0, 0, 0.001), drive));
  }
}
