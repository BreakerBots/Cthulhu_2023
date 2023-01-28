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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollower;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollowerConfig;
import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestWaypointAutoPath extends SequentialCommandGroup {
  /** Creates a new TestWaypointAutoPath. */
  public TestWaypointAutoPath(Drive drive) {
    ProfiledPIDController anglePID = new ProfiledPIDController(0.000000001, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
    PIDController xDrivePID = new PIDController(.5, 0, 0.0);
    PIDController yDrivePID = new PIDController(.5, 0, 0.0);
    HolonomicDriveController driveController = new HolonomicDriveController(xDrivePID, yDrivePID, anglePID);
    driveController.setTolerance(new Pose2d(0.15, 0., Rotation2d.fromDegrees(180)));
    
    BreakerWaypointPath wpp = new BreakerWaypointPath(
        new TrapezoidProfile.Constraints(0.5, 0.1), 
        new Translation2d(Units.feetToMeters(3), Units.feetToMeters(1))
        );

        BreakerWaypointPath wpp2 = new BreakerWaypointPath(
          new TrapezoidProfile.Constraints(0.1, 0.1), 
          new Translation2d(Units.feetToMeters(14), Units.feetToMeters(0)),
          new Translation2d(Units.feetToMeters(1.5), Units.feetToMeters(0)),
          new Translation2d(Units.feetToMeters(1.5), Units.feetToMeters(4))
        );
    BreakerSwerveWaypointFollowerConfig config = new BreakerSwerveWaypointFollowerConfig(drive, driveController);
    addCommands(
        new BreakerStartTrajectoryPath(drive, new Pose2d()),
        new BreakerSwerveWaypointFollower(config, wpp)
        // new WaitCommand(2),
        // new BreakerSwerveWaypointFollower(config, wpp2)
        );
  }
}
