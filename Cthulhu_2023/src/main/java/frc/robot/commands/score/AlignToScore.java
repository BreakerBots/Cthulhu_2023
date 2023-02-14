// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.score;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
import frc.robot.Node.NodeColumn;
import frc.robot.Node.NodeGroup;
import frc.robot.subsystems.Drive;

public class AlignToScore extends CommandBase {
  /** Creates a new AlignToScore. */
  private NodeColumn column;
  private Pose2d tgtPose;
  private BreakerHolonomicDriveController driveController;
  private Drive drive;
  public AlignToScore(NodeColumn column, Drive drive) {
    this.column = column;
    ProfiledPIDController anglePID = new ProfiledPIDController(2.25, 0.0, 0.0, new TrapezoidProfile.Constraints(9.0, 9.0));
    PIDController drivePID = new PIDController(1.0, 0, 0.0);
    driveController = new BreakerHolonomicDriveController(drivePID, anglePID);
    driveController.setTolerances(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(2)));
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tgtPose = column.getAlignmentPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveController.calculate(drive.getOdometryPoseMeters(), tgtPose, 1.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveController.atTargetPose();
  }
}
