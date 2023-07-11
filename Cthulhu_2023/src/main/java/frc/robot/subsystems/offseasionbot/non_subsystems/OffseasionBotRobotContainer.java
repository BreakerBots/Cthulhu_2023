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
import frc.robot.commands.offseasonbot.IntakeFromDoubleSubstation;
import frc.robot.commands.offseasonbot.IntakeFromGround;
import frc.robot.commands.offseasonbot.IntakeFromSingleSubstation;
import frc.robot.commands.offseasonbot.StowElevatorIntakeAssembly;
import frc.robot.commands.offseasonbot.TeleopScoreGamePiece;
import frc.robot.commands.offseasonbot.drive.TeleopSnapDriveToCardinalHeading;
import frc.robot.commands.offseasonbot.drive.TeleopSnapDriveToCardinalHeading.SwerveCardinal;
import frc.robot.commands.offseasonbot.intake.EjectGamePiece;
import frc.robot.commands.offseasonbot.intake.SetIntakeRollerState;
import frc.robot.commands.offseasonbot.intake.SetIntakeRollerState.IntakeRollerStateRequest;
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
    //drive controls
    driverControllerSys.getDPad().getUp().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.FRONT, drivetrainSys, teleopDriveController));
    driverControllerSys.getDPad().getLeft().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.LEFT, drivetrainSys, teleopDriveController));
    driverControllerSys.getDPad().getRight().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.RIGHT, drivetrainSys, teleopDriveController));
    driverControllerSys.getDPad().getDown().onTrue(new TeleopSnapDriveToCardinalHeading(SwerveCardinal.BACK, drivetrainSys, teleopDriveController));

    //scoreing controls
    operatorControlPadSys.getScoringCommandRequestTrigger().onTrue(new TeleopScoreGamePiece(operatorControlPadSys, drivetrainSys, elevatorSys));

    //stow elevator (driver controls: LB / RB = stow) (operator controls: 20 = stow)
    driverControllerSys.getLeftBumper()
    .or(driverControllerSys.getRightBumper())
    .or(operatorControlPadSys.getElevatorStowButton())
    .onTrue(new StowElevatorIntakeAssembly(elevatorSys, intakeSys, true));

    //intake from ground, (driver controls: X = cube, Y = cone) (operator controls: 14 = cube, 15 = cone)
    driverControllerSys.getButtonY().or(operatorControlPadSys.getIntakeGroundConeButton()).onTrue(new IntakeFromGround(elevatorSys, intakeSys, true, GamePieceType2.CONE));
    driverControllerSys.getButtonX().or(operatorControlPadSys.getIntakeGroundCubeButton()).onTrue(new IntakeFromGround(elevatorSys, intakeSys, true, GamePieceType2.CUBE));

    //intake from single sub, (operator controls: 9 = cube, 10 = cone)
    operatorControlPadSys.getIntakeSingleSubstationConeButton().onTrue(new IntakeFromSingleSubstation(elevatorSys, intakeSys, false, GamePieceType2.CONE));
    operatorControlPadSys.getIntakeSingleSubstationCubeButton().onTrue(new IntakeFromSingleSubstation(elevatorSys, intakeSys, false, GamePieceType2.CUBE));

    //intake from double sub, (operator controls: 4 = cube, 5 = cone)
    operatorControlPadSys.getIntakeDoubleSubstationConeButton().onTrue(new IntakeFromDoubleSubstation(elevatorSys, intakeSys, false, GamePieceType2.CONE));
    operatorControlPadSys.getIntakeDoubleSubstationCubeButton().onTrue(new IntakeFromDoubleSubstation(elevatorSys, intakeSys, false, GamePieceType2.CUBE));

    //intake roller controls, (driver controls: A = intake, B = stop) (operator controls: 19 = extake)
    driverControllerSys.getButtonA().onTrue(new SetIntakeRollerState(intakeSys, IntakeRollerStateRequest.INTAKE));
    driverControllerSys.getButtonB().onTrue(new SetIntakeRollerState(intakeSys, IntakeRollerStateRequest.STOP));
    operatorControlPadSys.getRollerExtakeButton().onTrue(new SetIntakeRollerState(intakeSys, IntakeRollerStateRequest.EXTAKE));

    //game piece eject, (operator controls: fire button)
    operatorControlPadSys.getEjectGamePieceButton().onTrue(new EjectGamePiece(intakeSys));

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
