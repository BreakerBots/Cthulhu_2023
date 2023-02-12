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

public class MoveToGamePiece extends CommandBase {
  /** Creates a new MoveToGamePiece. */
  public Drive drive;
  private GamePieceTracker tracker;
  private BreakerHolonomicDriveController driveController;
  private boolean hasRun = false;
  private boolean useRot;
  private int ind = 0;
  private Pose2d tgtPose;
  public MoveToGamePiece(Drive drive, GamePieceTracker tracker) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.tracker = tracker;
    ProfiledPIDController anglePID = new ProfiledPIDController(2.25, 0.0, 0.0, new TrapezoidProfile.Constraints(9.0, 9.0));
    PIDController drivePID = new PIDController(1.0, 0, 0.0);
    driveController = new BreakerHolonomicDriveController(drivePID, anglePID);
    driveController.setTolerances(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(2)));
    useRot = true;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (tracker.hasTargets()) {
    //   TrackedGamePiece piece = tracker.getBestTrackedGamePiece();
    //   Translation2d tgtTrans = piece.getRobotToTargetTranslation();
    //   tgtPose = new Pose2d(new Translation2d(tgtTrans.getNorm() - 0.05, tgtTrans.getAngle()).plus(drive.getOdometryPoseMeters().getTranslation()), new Rotation2d());
    //   System.out.println("Trans: " + tgtTrans + " |  Angle: " + piece.getTarget().getYaw() + " | SP: " + tgtPose);
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (tracker.hasTargets()) {
      hasRun = true;
      TrackedGamePiece piece = tracker.getBestTrackedGamePiece();
      Translation2d tgtTrans = piece.getRobotToTargetTranslation();
      tgtTrans = new Translation2d(tgtTrans.getNorm() - 0.25, tgtTrans.getAngle());
      if (tgtTrans.getNorm() < 0.5) {
        useRot = false;
      }
      tgtPose = new Pose2d(tgtTrans, new Rotation2d());

      Pose2d corPose = new Pose2d(new Translation2d(), useRot ? Rotation2d.fromDegrees(piece.getTarget().getYaw()) : new Rotation2d());

    

      ChassisSpeeds spd = driveController.calculate(corPose, tgtPose, 0.5);
      drive.move(spd.vxMetersPerSecond, spd.vyMetersPerSecond, spd.omegaRadiansPerSecond);
      if (ind++%25==0) {
       //System.out.println("DISTANCE: " + tgtPose.getTranslation().getNorm());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !tracker.hasTargets() || (driveController.atTargetPose() && hasRun);
  }
}
