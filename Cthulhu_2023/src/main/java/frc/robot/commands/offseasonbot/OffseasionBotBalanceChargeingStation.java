// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.offseasionbot.OffseasionBotDrive;

public class OffseasionBotBalanceChargeingStation extends CommandBase {
  /** Creates a new OffseasionBotBalanceChargeingStation. */
  private OffseasionBotDrive drive;
  private PPHolonomicDriveController driveController;
  public OffseasionBotBalanceChargeingStation(OffseasionBotDrive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
