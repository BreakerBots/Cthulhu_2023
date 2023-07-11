// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.offseasonbot.intake.SetIntakeRollerState;
import frc.robot.subsystems.offseasionbot.Elevator;
import frc.robot.subsystems.offseasionbot.Intake;
import frc.robot.subsystems.offseasionbot.Elevator.ElevatorTarget;
import frc.robot.subsystems.offseasionbot.Intake.ActuatorMotorState;
import frc.robot.subsystems.offseasionbot.Intake.RollerState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowElevatorIntakeAssembly extends SetElevatorIntakeAssemblyState {
  /** Creates a new StowElevatorIntakeAssembly. */
  public StowElevatorIntakeAssembly(Elevator elevator, Intake intake, boolean verifyIntakeAcutation) {
     super(elevator, intake, ElevatorTarget.STOW, ActuatorMotorState.RETRACTING, verifyIntakeAcutation);
  }
}
