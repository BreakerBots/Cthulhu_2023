// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.pose;

import static frc.robot.subsystems.SebArm.State.PICKUP_LOW_CUBE;
import static frc.robot.subsystems.SebArm.State.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerPoseWaypointPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwervePoseWaypointPathFollower;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.commands.BalanceChargingStation;
import frc.robot.commands.MoveArmToState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.SebArm;
import frc.robot.subsystems.SebArm.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GatePlace2 extends SequentialCommandGroup {
        public GatePlace2(Drive drive, BreakerPigeon2 imu, RollerIntake intake, SebArm arm) {

                // Move out to game piece
                BreakerPoseWaypointPath wpp = new BreakerPoseWaypointPath(
                                1.5,
                                new Pose2d(1.9, 1.02, new Rotation2d()),
                                new Pose2d(6.75, 0.885, Rotation2d.fromDegrees(180)));

                // Grab then move back to position
                BreakerPoseWaypointPath wpp1 = new BreakerPoseWaypointPath(
                                1.5,
                                new Pose2d(7.02, 0.885, Rotation2d.fromDegrees(180)),
                                new Pose2d(3.457, 0.885, Rotation2d.fromDegrees(180)),
                                new Pose2d(1.9, 1.02, Rotation2d.fromDegrees(180)));

                // Move to charging station
                // BUMP UP SPEED AT COMP
                // BreakerPoseWaypointPath wpp2 = new BreakerPoseWaypointPath(
                //                 1.5,
                //                 new Pose2d(1.9, 2.25, Rotation2d.fromDegrees(180)),
                //                 new Pose2d(4.2, 2.5, Rotation2d.fromDegrees(180)));

                addCommands(
                                new BreakerStartTrajectoryPath(drive,
                                                Drive.mirrorPathToAlliance(wpp).getWaypoints()[0]),
                                new MoveArmToState(arm, PLACE_CUBE_MID),
                                intake.ejectCmd(),
                                new WaitCommand(.25),
                                new MoveArmToState(arm, STOW_CUBE),
                                new ParallelCommandGroup(
                                        new BreakerSwervePoseWaypointPathFollower(drive.autoConfig, true,
                                        Drive.mirrorPathToAlliance(wpp)),
                                                new MoveArmToState(arm, PICKUP_LOW_CUBE),
                                                intake.startCmd()),
                                new WaitCommand(0.5),
                                intake.stopCmd(),
                                new ParallelCommandGroup(
                                                new MoveArmToState(arm, STOW_CUBE),
                                                new BreakerSwervePoseWaypointPathFollower(drive.autoConfig, true,
                                                                Drive.mirrorPathToAlliance(wpp1))),
                                intake.ejectCmd(),
                                new WaitCommand(0.25),
                                intake.stopCmd(),
                                new MoveArmToState(arm, STOW_CUBE)
                                // new BreakerSwervePoseWaypointPathFollower(drive.autoConfig, true,
                                //                 Drive.mirrorPathToAlliance(wpp2)),
                                // new BalanceChargingStation(drive, imu)
                                );
        }
}
