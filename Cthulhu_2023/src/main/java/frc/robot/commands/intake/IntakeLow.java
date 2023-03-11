// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmState;
import frc.robot.subsystems.arm.Arm.MoveToState;

public class IntakeLow extends CommandBase {
  /** Creates a new IntakeLow. */
  private MoveToState moveToState;
  private RollerIntake rollerIntake;
  private Arm arm;
  public IntakeLow(RollerIntake rollerIntake, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveToState = arm.new MoveToState(rollerIntake.isConeModeSelected() ? ArmState.PICKUP_CONE_LOW : ArmState.PICKUP_CUBE_LOW, arm);
    new InstantCommand(rollerIntake::runSelectedIntakeMode).schedule();
    moveToState.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return moveToState.isFinished();
  }
}
