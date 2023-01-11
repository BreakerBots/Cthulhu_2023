// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;

public class BalenceChargeingStation extends CommandBase {
  /** Creates a new BalenceChargeingStation. */
  private BreakerPigeon2 imu;
  private PIDController balencePID;
  public BalenceChargeingStation(BreakerPigeon2 imu) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.imu = imu;
    balencePID = new PIDController(0, 0, 0);
    balencePID.setTolerance(0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
