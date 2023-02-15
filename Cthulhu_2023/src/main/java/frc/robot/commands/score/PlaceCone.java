// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.score;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Node.NodeLevel;
import frc.robot.commands.intake.OpenGripper;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.MoveToState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCone extends SequentialCommandGroup {
  /** Creates a new PlaceHighCone. */
  public PlaceCone(NodeLevel level, Arm arm, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    switch(level) {
      case HIGH:
        addCommands(
          arm.new MoveToState(Arm.ArmState.PLACE_HIGH_CONE),
          new OpenGripper(gripper)
        );
        break;
      case MIDDLE:
        addCommands(
          arm.new MoveToState(Arm.ArmState.PLACE_MEDIUM_CONE),
          new OpenGripper(gripper)
        );
        break;
      case LOW:
        addCommands(
          arm.new MoveToState(Arm.ArmState.PLACE_HYBRID),
          new OpenGripper(gripper)
        );
        break;
    }
  }
}
