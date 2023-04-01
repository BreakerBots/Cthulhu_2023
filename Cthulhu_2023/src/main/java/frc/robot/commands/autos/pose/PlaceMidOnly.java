// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.pose;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.MoveArmToState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.SebArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceMidOnly extends SequentialCommandGroup {
  /** Creates a new PlaceMidOnly. */
  public PlaceMidOnly(Drive drive, SebArm arm, RollerIntake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drive.setOdometryRotation(DriverStation.getAlliance() == Alliance.Red ? Rotation2d.fromDegrees(180) : new Rotation2d()), drive),
      new MoveArmToState(arm, SebArm.State.PLACE_CUBE_MID),
      intake.ejectCmd(),
      new WaitCommand(0.25),
      intake.stopCmd()
    );
  }
}
