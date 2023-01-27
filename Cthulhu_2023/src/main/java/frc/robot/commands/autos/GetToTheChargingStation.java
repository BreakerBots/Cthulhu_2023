// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartAutoPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollower;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollowerConfig;
import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetToTheChargingStation extends SequentialCommandGroup {
    public GetToTheChargingStation(Drive drive) {
        HolonomicDriveController driveController = new HolonomicDriveController(
            new PIDController(2.0, 0.0, 0.1),
            new PIDController(2.0, 0.0, 0.1),
            new ProfiledPIDController(
                0.000000001, 0.0,0.0,
                new TrapezoidProfile.Constraints(
                    0.05,
                    0.05
                )
            )
        );
        driveController.setTolerance(new Pose2d(0.5, 0.5, new Rotation2d(2*Math.PI)));
        BreakerWaypointPath wpp = new BreakerWaypointPath(
            new TrapezoidProfile.Constraints(0.5, 0.05),
            new Translation2d(0, Units.feetToMeters(4)),
            new Translation2d(Units.feetToMeters(14), Units.feetToMeters(4))
        );
        BreakerSwerveWaypointFollowerConfig config = new BreakerSwerveWaypointFollowerConfig(drive, driveController);
        addRequirements(drive);
        addCommands(
            // Start at (0, 0) with current angle
                new BreakerStartAutoPath(drive, new Pose2d(new Translation2d(), drive.getOdometryPoseMeters().getRotation())),
                new BreakerSwerveWaypointFollower(config, wpp)
                );
    }
}
