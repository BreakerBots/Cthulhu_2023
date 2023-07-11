// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.offseasonbot.intake.SetIntakeRollerState;
import frc.robot.commands.offseasonbot.intake.SetIntakeRollerState.IntakeRollerStateRequest;
import frc.robot.subsystems.offseasionbot.Elevator;
import frc.robot.subsystems.offseasionbot.Intake;
import frc.robot.subsystems.offseasionbot.Elevator.ElevatorTarget;
import frc.robot.subsystems.offseasionbot.Intake.ActuatorMotorState;
import frc.robot.subsystems.offseasionbot.non_subsystems.GamePieceType2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromDoubleSubstation extends ParallelCommandGroup {
  /** Creates a new IntakeFromGround. */
  public IntakeFromDoubleSubstation(Elevator elevator, Intake intake, boolean verifyIntakeAcutation, GamePieceType2 gamePieceType) {
    addCommands(
      new SetElevatorIntakeAssemblyState(elevator, intake, gamePieceType.isCube() ? ElevatorTarget.PICKUP_DOUBLE_SUBSTATION_CUBE : ElevatorTarget.PICKUP_DOUBLE_SUBSTATION_CONE, ActuatorMotorState.EXTENDING, verifyIntakeAcutation),
      new SetIntakeRollerState(intake, IntakeRollerStateRequest.INTAKE)
    );
  }
}
