// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.offseasionbot.Elevator;

public class ElevatorMoveToHight extends CommandBase {
  /** Creates a new ElevatorMoveToHight. */
  private Elevator elevator;
  private double targetHightMeters;
  public ElevatorMoveToHight(Elevator elevator, double targetHightMeters) {
    this.elevator = elevator;
    this.targetHightMeters = targetHightMeters;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setTarget(targetHightMeters);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atTargetHeight();
  }
}
