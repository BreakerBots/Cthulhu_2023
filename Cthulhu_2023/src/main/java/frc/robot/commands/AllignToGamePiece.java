// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.GamePieceTracker;
import frc.robot.subsystems.GamePieceTracker.TrackedGamePiece;

public class AllignToGamePiece extends CommandBase {
  /** Creates a new MoveToGamePiece. */
  public Drive drive;
  private GamePieceTracker tracker;
  private boolean hasRun = false;
  private PIDController anglePID;
  private double yaw = 999999.0;
  public AllignToGamePiece(Drive drive, GamePieceTracker tracker) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.tracker = tracker;
    anglePID = new PIDController(0.065, 0.0, 0.0);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (tracker.hasTargets()) {
      hasRun = true;
      TrackedGamePiece piece = tracker.getBestTrackedGamePiece();
      drive.move(0.0, 0, anglePID.calculate(piece.getTarget().getYaw(), 0.0));
      System.out.println("ERR: " + anglePID.getPositionError() + "| V-ERR: " + anglePID.getVelocityError());
      yaw = piece.getTarget().getYaw();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !tracker.hasTargets() || Math.abs(yaw) < 2.0;
  }
}
