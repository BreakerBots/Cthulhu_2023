// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.commands.offseasonbot.drive.MoveToPose;
import frc.robot.commands.offseasonbot.elevator.ElevatorMoveToHight;
import frc.robot.commands.offseasonbot.intake.EjectGamePiece;
import frc.robot.commands.offseasonbot.rumble.DoublePulseRumble;
import frc.robot.commands.offseasonbot.rumble.SinglePulseRumble;
import frc.robot.commands.offseasonbot.rumble.TriplePulseRumble;
import frc.robot.subsystems.offseasionbot.Elevator;
import frc.robot.subsystems.offseasionbot.Intake;
import frc.robot.subsystems.offseasionbot.OffseasionBotDrive;
import frc.robot.subsystems.offseasionbot.Elevator.ElevatorTarget;
import frc.robot.subsystems.offseasionbot.non_subsystems.Node;
import frc.robot.subsystems.offseasionbot.non_subsystems.OperatorControlPad;
import frc.robot.subsystems.offseasionbot.non_subsystems.Node.NodeType;
import frc.robot.subsystems.offseasionbot.non_subsystems.OffseasionBotConstants.DriveConstants;
import frc.robot.subsystems.offseasionbot.non_subsystems.OffseasionBotConstants.ScoreingConstants;

public class TeleopScoreGamePiece extends CommandBase {
  /** Creates a new ScoreGamePiece. */
  private Node selectedNode;
  private OperatorControlPad operatorControlPad;
  private BreakerXboxController driverController;
  private OffseasionBotDrive drivetrain;
  private Elevator elevator;
  private Intake intake;
  private SequentialCommandGroup scoreingSequince;
  public TeleopScoreGamePiece(OperatorControlPad operatorControlPad, BreakerXboxController driverController, OffseasionBotDrive drivetrain, Elevator elevator, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BreakerLog.logEvent("TeleopScoreGamePiece instance started");
    Optional<Node> selectedNodeOptional = operatorControlPad.getSelectedScoringNode();
    if (selectedNodeOptional.isPresent()) {
      selectedNode = selectedNodeOptional.get();
      ElevatorTarget elevatorTgt = getElevatorTarget();
      Pose2d allignmentPose = selectedNode.getAllignmentPose();
      BreakerLog.logEvent(String.format("TeleopScoreGamePiece instance selected node indentified, scoreing sequince starting (node: %s) (elevator tgt: %s) (allignment pose: %s)", selectedNode.toString(), elevatorTgt.toString(), allignmentPose.toString()));
      new SinglePulseRumble(driverController).schedule();
      scoreingSequince = 
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new MoveToPose(selectedNode.getAllignmentPose(), ScoreingConstants.TELEOP_SCOREING_MOVE_TO_POSE_MAX_LINEAR_VEL, drivetrain), 
          new ElevatorMoveToHight(elevator, getElevatorTarget())
        ),
        new ConditionalCommand(new EjectGamePiece(intake), new InstantCommand(this::cancel), () -> preEjectCheck(elevatorTgt)));
    } else {
      this.cancel();
      BreakerLog.logEvent("TeleopScoreGamePiece instance FAILED, no node selected");
    }
  
  }

  private boolean preEjectCheck(ElevatorTarget elevatorTarget) {
    boolean check = (elevator.atTargetHeight() && (elevator.getTargetHeightMeters() == elevatorTarget.getTargetHeight())) && BreakerMath.epsilonEqualsPose2d(selectedNode.getAllignmentPose(), drivetrain.getOdometryPoseMeters(), DriveConstants.BHDC_POSE_TOLERENCE);
    if (!check) {
      BreakerLog.logEvent("TeleopScoreGamePiece instance FAILED, game piece eject procedure pre init check failed, elevator or drive not at desired states");
    } else {
      BreakerLog.logEvent("TeleopScoreGamePiece instance game piece eject procedure pre init check PASSED, begining eject");
    }
    return check;

  }
 
  private ElevatorTarget getElevatorTarget() {
    switch (selectedNode.getHeight()) {
      case HIGH:
        if (selectedNode.getType() == NodeType.CONE) {
          return ElevatorTarget.PLACE_CONE_HIGH;
        }
        return ElevatorTarget.PLACE_CUBE_HIGH;
      case MID:
        if (selectedNode.getType() == NodeType.CONE) {
          return ElevatorTarget.PLACE_CONE_MID;
        }
        return ElevatorTarget.PLACE_CUBE_MID;
      case LOW:
      default:
        return ElevatorTarget.PLACE_HYBRID;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (Objects.nonNull(scoreingSequince) && scoreingSequince.isScheduled())  {
      scoreingSequince.cancel();
    }
    if (interrupted) {
      new TriplePulseRumble(driverController).schedule();
    } else {
      new DoublePulseRumble(driverController).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Objects.nonNull(scoreingSequince)) {
      return scoreingSequince.isFinished();
    }
    return false;
  }
}
