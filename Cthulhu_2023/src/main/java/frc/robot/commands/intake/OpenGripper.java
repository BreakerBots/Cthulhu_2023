// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GamePieceType;
import frc.robot.subsystems.Gripper;

public class OpenGripper extends CommandBase {
  /** Creates a new OpenGripper. */
  private Gripper gripper;
  private GamePieceType initialGamePiece;
  public OpenGripper(Gripper gripper) {
    addRequirements(gripper);
    this.gripper = gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialGamePiece  = gripper.getControlledGamePieceType();
    gripper.setOpenGrip();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripper.isOpen() && (initialGamePiece == GamePieceType.NONE || initialGamePiece != gripper.getControlledGamePieceType());
  }
}