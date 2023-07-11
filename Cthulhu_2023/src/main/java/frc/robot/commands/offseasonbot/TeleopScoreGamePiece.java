// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.commands.offseasonbot.drive.MoveToPose;
import frc.robot.subsystems.offseasionbot.Elevator;
import frc.robot.subsystems.offseasionbot.OffseasionBotDrive;
import frc.robot.subsystems.offseasionbot.non_subsystems.Node;
import frc.robot.subsystems.offseasionbot.non_subsystems.OperatorControlPad;

public class TeleopScoreGamePiece extends CommandBase {
  /** Creates a new ScoreGamePiece. */
  private Node selectedNode;
  private OperatorControlPad operatorControlPad;
  private OffseasionBotDrive drivetrain;
  private Elevator elevator;
  private SequentialCommandGroup scoreingSequince;
  public TeleopScoreGamePiece(OperatorControlPad operatorControlPad, OffseasionBotDrive drivetrain, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BreakerLog.logEvent("TeleopScoreGamePiece instance started");
    Optional<Node> selectedNodeOptional = operatorControlPad.getSelectedScoringNode();
    if (selectedNodeOptional.isPresent()) {
      selectedNode = selectedNodeOptional.get();
      BreakerLog.logEvent(String.format("TeleopScoreGamePiece instance selected node indentified, scoreing sequince starting (node: %s)", selectedNode.toString()));

      selectedNode.ge


      scoreingSequince = new SequentialCommandGroup(new MoveToPose(selectedNode.getAllignmentPose(), drivetrain), , );
    } else {
      this.cancel();
      BreakerLog.logEvent("TeleopScoreGamePiece instance FAILED, no node selected");
    }
  
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return scoreingSequince.isFinished();
  }
}
