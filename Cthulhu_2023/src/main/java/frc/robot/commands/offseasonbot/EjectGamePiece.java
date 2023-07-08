// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.offseasonbot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.subsystems.offseasionbot.Intake;
import frc.robot.subsystems.offseasionbot.Intake.ActuatorState;
import frc.robot.subsystems.offseasionbot.non_subsystems.OffseasionBotConstants.IntakeConstants;

public class EjectGamePiece extends SequentialCommandGroup {
  /** Creates a new EjectGamePiece. */
  public EjectGamePiece(Intake intake) {
    addCommands(
      new InstantCommand(() -> BreakerLog.logSuperstructureEvent("GAME PIECE EJECTION PROCEDURE STARTED")),
      new InstantCommand(intake::extend, intake),
      new ParallelRaceGroup(new WaitCommand(IntakeConstants.EJECT_COMMAND_WAIT_FOR_EXTEND_TIMEOUT), new WaitUntilCommand(() -> intake.getActuatorState() == ActuatorState.EXTENDED)),
      new InstantCommand(intake::extake, intake),
      new ParallelRaceGroup(new SequentialCommandGroup(new WaitUntilCommand(() -> !intake.hasGamePiece()), new WaitCommand(IntakeConstants.EJECT_COMMAND_CUTOFF_TRALING_DELAY)), new WaitCommand(IntakeConstants.EJECT_COMMAND_CUTOFF_TIMEOUT)),
      new InstantCommand(intake::stopRoller, intake),
      new InstantCommand(() -> {if (intake.hasGamePiece()) { BreakerLog.logSuperstructureEvent("GAME PIECE EJECTION PROCEDURE FAILED, GAME PIECE NOT PROPERLY EJECTED, COMMMAND TIMED OUT"); } else { BreakerLog.logSuperstructureEvent("GAME PIECE EJECTION PROCEDURE SUCESSFULL, GAME PIECE EJECTED");}})
    );
  }
}
