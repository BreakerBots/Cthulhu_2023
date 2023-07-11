// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot.non_subsystems;


import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import frc.robot.commands.offseasonbot.StowElevatorIntakeAssembly;
import frc.robot.commands.offseasonbot.TeleopScoreGamePiece;
import frc.robot.commands.offseasonbot.drive.TeleopSnapDriveToCardinalHeading;
import frc.robot.commands.offseasonbot.drive.TeleopSnapDriveToCardinalHeading.SwerveCardinal;
import frc.robot.subsystems.offseasionbot.Elevator;
import frc.robot.subsystems.offseasionbot.Intake;
import frc.robot.subsystems.offseasionbot.OffseasionBotDrive;
import frc.robot.subsystems.offseasionbot.Vision;
import frc.robot.subsystems.offseasionbot.non_subsystems.OffseasionBotConstants.MiscConstants;
import frc.robot.subsystems.offseasionbot.non_subsystems.OffseasionBotConstants.OperatorConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class OffseasionBotRobotContainer {
    private static final BreakerXboxController driverControllerSys = new BreakerXboxController(0);
    private static final OperatorControlPad operatorControlPadSys = new OperatorControlPad(OperatorConstants.OPERATOR_PAD_PORT);

    private static final Vision visionSys = new Vision();
    private static final BreakerPigeon2 imuSys = new BreakerPigeon2(MiscConstants.IMU_ID, MiscConstants.CANIVORE_1);

    private static final OffseasionBotDrive drivetrainSys = new OffseasionBotDrive(imuSys, visionSys);
    private static final Elevator elevatorSys = new Elevator();
    private static final Intake intakeSys = new Intake();
  
    private static final BreakerTeleopSwerveDriveController teleopDriveController = new BreakerTeleopSwerveDriveController(drivetrainSys, driverControllerSys);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public OffseasionBotRobotContainer() {
    drivetrainSys.setDefaultCommand(teleopDriveController);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //driver controls
    driverControllerSys.getDPad().getUp().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.FRONT, drivetrainSys, teleopDriveController));
    driverControllerSys.getDPad().getLeft().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.LEFT, drivetrainSys, teleopDriveController));
    driverControllerSys.getDPad().getRight().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.RIGHT, drivetrainSys, teleopDriveController));
    driverControllerSys.getDPad().getDown().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.BACK, drivetrainSys, teleopDriveController));

    driverControllerSys.getLeftBumper().or(driverControllerSys.getRightBumper()).onTrue(new StowElevatorIntakeAssembly(elevatorSys, intakeSys, true));

    driverControllerSys.

    //operator controls
    operatorControlPadSys.getScoringCommandRequestTrigger().onTrue(new TeleopScoreGamePiece(operatorControlPadSys, drivetrainSys, elevatorSys));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
