// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.GamePieceTracker;

public class DriveForwardToGamePiece extends CommandBase {
  /** Creates a new DriveForward. */
  private Drive drive;
  private PIDController pid;
  private GamePieceTracker tracker;
  private double tgtDist = 0;
  private Pose2d initPose;
  public DriveForwardToGamePiece(Drive drive, GamePieceTracker tracker) {
    addRequirements(drive);
    this.drive = drive;
    this.tracker = tracker;
    pid = new PIDController(1.0, 0, 0.0);
    pid.setTolerance(0.01, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPose = drive.getOdometryPoseMeters();
    tgtDist = tracker.getBestTrackedGamePiece().getDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (tracker.hasTargets()) {
      drive.move(pid.calculate((-tgtDist) + drive.getOdometryPoseMeters().getTranslation().getDistance(initPose.getTranslation()), 0.15), 0, 0);
      System.out.println("ERR: " + pid.getPositionError() + "| V-ERR: " + pid.getVelocityError());

      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs((-tgtDist
    ) + drive.getOdometryPoseMeters().getTranslation().getDistance(initPose.getTranslation())) <= 0.2 || !tracker.hasTargets();
  }
}
