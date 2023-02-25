// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import org.opencv.aruco.GridBoard;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.gamepiece.GamePieceType;
import frc.robot.subsystems.Gripper;


/** Gripper default command */
public class RunGripper extends CommandBase {
  /** Creates a new RunGripper. */
  private Gripper gripper;
  private GamePieceType prevType;
  private final Timer changeTimer = new Timer();
  private boolean hasAutoClosed = false;
  public RunGripper(Gripper gripper) {
    this.gripper = gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevType = gripper.getControlledGamePieceType();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GamePieceType curType = gripper.getControlledGamePieceType();
    if (prevType != curType && curType != GamePieceType.NONE && gripper.isOpen()) {
      changeTimer.reset();
      changeTimer.start();
      hasAutoClosed = false;
    } else if (changeTimer.hasElapsed(Constants.GripperConstants.AUTO_CLOSE_DELAY_SEC) && !hasAutoClosed) {
      gripper.setClosedGrip();
      hasAutoClosed = true;
    }
    prevType = curType;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
