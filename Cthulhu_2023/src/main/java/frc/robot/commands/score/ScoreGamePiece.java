// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.score;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Node;
import frc.robot.Node.NodeColumn;
import frc.robot.Node.NodeLevel;
import frc.robot.Node.NodeType;
import frc.robot.RobotContainer;
import frc.robot.BreakerLib.driverstation.gamepad.BreakerGamepadTimedRumbleCommand;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerDPad;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gamepiece.GamePieceType;

/** Command to score the given game piece into the desired goal. */
public class ScoreGamePiece extends CommandBase {
  /** Creates a new InitScore. */
  private Drive drive;
  private Arm arm;
  private Gripper gripper;
  private BreakerXboxController controller;
  private NodeColumn targetCol;
  private boolean targetSelected;
  private PlaceGamePiece placeCom;
  private AlignToScore alignCom;
  private Node targetNode;
  private GamePieceType pieceType;
  
  public ScoreGamePiece(Drive drive, Arm arm, Gripper gripper, BreakerXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.gripper = gripper;
    this.drive = drive;
    this.controller = controller;
    targetSelected = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pos = drive.getOdometryPoseMeters();
    NodeColumn closestCol = Node.COLUMNS[0];
    double closestDist = closestCol.getAlignmentPose().getTranslation().getDistance(pos.getTranslation());
    for (NodeColumn col: Node.COLUMNS) {
      double dist = col.getAlignmentPose().getTranslation().getDistance(pos.getTranslation());
      if (dist < closestDist) {
        closestCol = col;
        closestDist = dist;
      }
    }
    targetCol = closestCol;
    alignCom = new AlignToScore(closestCol, drive);
    alignCom.schedule();
    pieceType = gripper.getControlledGamePieceType();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!targetSelected) {
      BreakerDPad dp = controller.getDPad();
      boolean high = dp.getUp().getAsBoolean();
      boolean mid = dp.getLeft().getAsBoolean() || dp.getRight().getAsBoolean();
      boolean low =  dp.getDown().getAsBoolean();
      if (high || mid || low) {
        if (high) {
          targetNode = Node.fromColumnAndLevel(targetCol, NodeLevel.HIGH);
        } else if (mid) {
          targetNode = Node.fromColumnAndLevel(targetCol, NodeLevel.MIDDLE);
        } else if (low) {
          targetNode =  Node.fromColumnAndLevel(targetCol, NodeLevel.LOW);
        }
        NodeType tgtType = targetNode.getNodeType();
        if (
          (tgtType == NodeType.CONE && pieceType == GamePieceType.CONE) ||
          (tgtType == NodeType.CUBE && pieceType == GamePieceType.CUBE) ||
          tgtType == NodeType.HYBRID
          ) {
            placeCom = new PlaceGamePiece(targetNode, arm, gripper);
            targetSelected = true;
          } else {
            new BreakerGamepadTimedRumbleCommand(controller, 0.5, 0.5, 0.5).schedule();;
          }
      }
    } else {
      if (alignCom.isFinished()) {
        placeCom.schedule();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (targetSelected && placeCom.isScheduled()) {
      placeCom.cancel();
    }

    if (alignCom.isScheduled()) {
      alignCom.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.isGlobalManualOverride() || (targetSelected && placeCom.isFinished()) || pieceType == GamePieceType.NONE;
  }
}
