// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.offseasonbot.elevator.ElevatorMoveToHight;
import frc.robot.commands.offseasonbot.intake.SetIntakeActuatorMotorState;
import frc.robot.subsystems.offseasionbot.Elevator;
import frc.robot.subsystems.offseasionbot.Intake;
import frc.robot.subsystems.offseasionbot.Elevator.ElevatorTarget;
import frc.robot.subsystems.offseasionbot.Intake.ActuatorMotorState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorIntakeAssemblyState extends ParallelCommandGroup {
  /** Creates a new SetElevatorIntakeAssemblyState. */
  public SetElevatorIntakeAssemblyState(Elevator elevator, Intake intake, ElevatorTarget elevatorTarget, ActuatorMotorState intakeActuatorstate, boolean verifyIntakeAcutation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorMoveToHight(elevator, elevatorTarget),
      new SetIntakeActuatorMotorState(intake, intakeActuatorstate, !verifyIntakeAcutation)
    );
  }
}
