// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.GamePieceTracker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToGamePieceGroup extends SequentialCommandGroup {
  /** Creates a new MoveToGamePieceGroup. */
  public MoveToGamePieceGroup(Drive drive, GamePieceTracker tracker) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignToGamePiece(drive, tracker),
      new DriveForwardToGamePiece(drive, tracker)
    );
  }
}