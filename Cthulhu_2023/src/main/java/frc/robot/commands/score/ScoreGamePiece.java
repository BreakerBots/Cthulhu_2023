// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Node;
import frc.robot.Node.NodeType;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.arm.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreGamePiece extends SequentialCommandGroup {
  /** Creates a new ScoreGamePiece. */
  public ScoreGamePiece(Drive drive, Arm arm, Gripper gripper, Node targetNode) {
    addCommands(
      new AlignToScore(targetNode.getColumn(), drive),
      (targetNode.getNodeType() == NodeType.CONE ? new PlaceCone(targetNode.getLevel(), arm, gripper) : new PlaceCube(targetNode.getLevel(), arm, gripper))
    );
  }
}
