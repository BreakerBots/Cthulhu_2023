// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.pathplanner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.auto.pathplanner.BreakerSwervePathFollower;
import frc.robot.BreakerLib.auto.pathplanner.BreakerSwervePathFollower.BreakerSwervePathFollowerConfig;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartAutoPath;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestPath extends SequentialCommandGroup {
  /** Creates a new TestPath. */
  public TestPath(Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("test path", new PathConstraints(1.75, 3));
    // BreakerSwervePathFollowerConfig config = new BreakerSwervePathFollowerConfig(drive, new PPHolonomicDriveController(
    //     new PIDController(4.5, 0.0, 0), new PIDController(4.5, 0.0, 0), new PIDController(4.75, 0.0, 0.0)), false);
    addCommands(
        new BreakerStartAutoPath(drive, new Pose2d(examplePath.getInitialState().poseMeters.getTranslation(), examplePath.getInitialState().holonomicRotation)),
        //new BreakerSwervePathFollower(new BreakerSwervePathFollowerConfig(drive, drive.getConfig().getDriveController(), false), examplePath, true),
        drive.followPathCommand(examplePath)
        );
  }
}
