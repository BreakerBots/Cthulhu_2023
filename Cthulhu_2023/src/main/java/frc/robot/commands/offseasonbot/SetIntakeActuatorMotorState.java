// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.offseasionbot.Intake;
import frc.robot.subsystems.offseasionbot.Intake.ActuatorMotorState;

public class SetIntakeActuatorMotorState extends CommandBase {
  /** Creates a new SetIntakeActuatorMotorState. */
  private Intake intake;
  private ActuatorMotorState motorState; 
  private boolean isInstant;
  public SetIntakeActuatorMotorState(Intake intake, ActuatorMotorState motorState, boolean isInstant) {
   this.intake = intake;
   this.motorState = motorState;
   this.isInstant = isInstant;
   addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setActuatorMotorState(motorState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isInDesiredActuatorState() || isInstant;
  }
}
