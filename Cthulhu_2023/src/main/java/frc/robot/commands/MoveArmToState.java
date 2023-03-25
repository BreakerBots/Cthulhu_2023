// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SebArm;
import frc.robot.subsystems.SebArm.State;

public class MoveArmToState extends CommandBase {
  /** Creates a new MoveArmToState. */
  private SebArm arm;
  private State targetState;
  public MoveArmToState(SebArm arm, State targetState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.targetState = targetState;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmState(targetState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isAtTarget();
  }
}
