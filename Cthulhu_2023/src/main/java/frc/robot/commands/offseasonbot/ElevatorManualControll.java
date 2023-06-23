// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.offseasionbot.Elevator;
import frc.robot.subsystems.offseasionbot.Elevator.ElevatorConstants;

public class ElevatorManualControll extends CommandBase {
  /** Creates a new ElevatorManualControll. */
  private Elevator elevator;
  private ControlType controlType;
  private DoubleSupplier percentageSupplier;
  public ElevatorManualControll(Elevator elevator, ControlType controlType, DoubleSupplier percentageSupplier) {
    this.elevator = elevator;
    this.controlType = controlType;
    this.percentageSupplier = percentageSupplier;
    addRequirements(elevator);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controlType == ControlType.POSITION) {
      elevator.setTarget(percentageSupplier.getAsDouble() * ElevatorConstants.MAX_HEIGHT);
    } else {
      elevator.setManual(percentageSupplier.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static enum ControlType {
    POSITION,
    DUTY_CYCLE
  }
}
