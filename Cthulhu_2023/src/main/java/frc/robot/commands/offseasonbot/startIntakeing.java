// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.offseasionbot.Intake;

public class startIntakeing extends CommandBase {
  /** Creates a new startIntakeing. */
  private Intake intake;
  public startIntakeing(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.extend();
    intake.setRollerIntakeing();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
