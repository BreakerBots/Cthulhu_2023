// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerLegacySwerveDrive;

public class LockOntoAprilTag extends CommandBase {
  private BreakerLegacySwerveDrive drive;
  //private AprilTagTracker aprilTagTracker;

  public LockOntoAprilTag(BreakerLegacySwerveDrive drive) {
    addRequirements(drive);
    this.drive = drive;
    //this.aprilTagTracker = aprilTagTracker;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(0, 0, 0.3);
    System.out.println(Math.toRadians(drive.getBaseGyro().getRawYawRate()));
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
