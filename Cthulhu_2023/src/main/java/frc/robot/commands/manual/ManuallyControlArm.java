// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPose;

public class ManuallyControlArm extends CommandBase {

  private BreakerXboxController controller;
  private boolean isProximal = true;
  private Arm arm;

  /** Creates a new ManualyControlArm. */
  public ManuallyControlArm(BreakerXboxController controller, Arm arm) {
    addRequirements(arm);
    this.arm = arm;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.getLeftBumper().onTrue(new InstantCommand(() -> isProximal = !isProximal));
    double input = controller.getRightTrigger().get() - controller.getLeftTrigger().get();
    double desiredDelta = input * 2.0 / 50.0;
    ArmPose armPose = arm.getArmPose();
    if (isProximal) {
      arm.setManualTargetPose(
          new ArmPose(armPose.getShoulderAngle().plus(Rotation2d.fromDegrees(desiredDelta)), 
          armPose.getElbowAngle().plus(Rotation2d.fromDegrees(desiredDelta)))
        );
    } else {
      arm.setManualTargetPose(
          new ArmPose(armPose.getShoulderAngle().plus(Rotation2d.fromDegrees(desiredDelta)), armPose.getElbowAngle())
        );
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.isGlobalManualOverride();
  }
}

