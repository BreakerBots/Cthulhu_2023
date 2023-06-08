// Prototyping a new example RobotContainer setup. 

// KRAKEN BRANCH - NOT WORKING!

package frc.robot;

import static frc.robot.Constants.MiscConstants.*;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadAnalogDeadbandConfig;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import frc.robot.BreakerLib.util.math.functions.BreakerBezierCurve;
import frc.robot.BreakerLib.util.robot.BreakerRobotConfig;
import frc.robot.BreakerLib.util.robot.BreakerRobotManager;
import frc.robot.BreakerLib.util.robot.BreakerRobotStartConfig;
import frc.robot.commands.BalanceChargingStation;
import frc.robot.commands.autos.LeaveOnly;
import frc.robot.commands.autos.GateLeaveThenBalance;
import frc.robot.commands.autos.GatePlaceLeaveThenBalance;
import frc.robot.commands.autos.MidGateLeaveThenBalance;
import frc.robot.commands.autos.SubLeaveThenBalance;
import frc.robot.commands.autos.TESTPATH;
//import frc.robot.commands.autos.TESTPATH;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static final BreakerXboxController controllerSys = new BreakerXboxController(0);

  private final BreakerPigeon2 imuSys = new BreakerPigeon2(IMU_ID, CANIVORE_1);
  private final Drive drivetrainSys = new Drive(imuSys);
  private final BreakerBezierCurve driveCurve = new BreakerBezierCurve(new Translation2d(0.707, 0.186),
      new Translation2d(0.799, 0.317));
  private final BreakerTeleopSwerveDriveController manualDriveCommand = new BreakerTeleopSwerveDriveController(
      drivetrainSys, controllerSys).addSpeedCurves(driveCurve, driveCurve);

  private final Arm armSys = new Arm();
  private final RollerIntake rollerIntake = new RollerIntake();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    PhotonCamera.setVersionCheckEnabled(false);

    robotManagerSetup();

    // 0.06 is normal, 0.1 is for testing with bad controller
    controllerSys.configDeadbands(new BreakerGamepadAnalogDeadbandConfig(0.1, 0.1, 0.1, 0.1));

    drivetrainSys.resetOdometryPosition();

    configureButtonBindings();
    drivetrainSys.setDefaultCommand(manualDriveCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controllerSys.getButtonX().onTrue(new ParallelCommandGroup(new InstantCommand(rollerIntake::runSelectedIntakeMode),
        armSys.new MoveToState(ArmState.PICKUP_LOW, armSys)));
    controllerSys.getButtonY().onTrue(new ParallelCommandGroup(new InstantCommand(rollerIntake::runSelectedIntakeMode),
        armSys.new MoveToState(ArmState.PICKUP_HIGH, armSys)));
    controllerSys.getButtonA().onTrue(new InstantCommand(rollerIntake::eject));
    controllerSys.getButtonB().onTrue(new InstantCommand(rollerIntake::stop));
    controllerSys.getLeftBumper().or(controllerSys.getRightBumper()).onTrue(
        new ParallelCommandGroup(new InstantCommand(rollerIntake::stop), armSys.new MoveToState(ArmState.CARRY, armSys)));

    controllerSys.getDPad().getUp().onTrue(armSys.new MoveToState(ArmState.PLACE_HIGH, armSys));
    controllerSys.getDPad().getLeft().or(controllerSys.getDPad().getRight())
        .onTrue(armSys.new MoveToState(ArmState.PLACE_MEDIUM, armSys));
    controllerSys.getDPad().getDown().onTrue(armSys.new MoveToState(ArmState.PLACE_HYBRID, armSys));
    // controllerSys.getStartButton().onTrue()

    // ASK NIKO FIRST!!!
    controllerSys.getBackButton().onTrue(new InstantCommand(drivetrainSys::resetOdometryRotation));

    controllerSys.getStartButton().onTrue(new InstantCommand(rollerIntake::toggleConeModeSelected));
  }

  private void robotManagerSetup() {
    BreakerRobotConfig robotConfig = new BreakerRobotConfig(new BreakerRobotStartConfig(5104, "BreakerBots",
        "Cthulhu", 2023, "v1", "Yousif Alkhalaf, Roman Abrahamson, Sebastian Rueda"));

    BreakerRobotManager.setup(drivetrainSys, robotConfig);
  }

  public static boolean isGlobalManualOverride() {
    return controllerSys.getStartButton().getAsBoolean();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int pathNum = 0;
    switch (pathNum) {
      case 0:
       return new GateLeaveThenBalance(drivetrainSys, imuSys);
      case 1:
        return new MidGateLeaveThenBalance(drivetrainSys, imuSys);
      case 2:
      return null;
        //return new SubLeaveThenBalance(drivetrainSys, imuSys);
      case 3:
        return new GatePlaceLeaveThenBalance(drivetrainSys, armSys, rollerIntake, imuSys);
      case 4:
        return new LeaveOnly(drivetrainSys, imuSys);
      case 5:
        return new TESTPATH(drivetrainSys, imuSys);
      default:
        return null;

    }
  }
}
