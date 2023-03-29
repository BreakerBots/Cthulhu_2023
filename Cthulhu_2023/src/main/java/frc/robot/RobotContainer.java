// Prototyping a new example RobotContainer setup. 

package frc.robot;

import static frc.robot.Constants.MiscConstants.CANIVORE_1;
import static frc.robot.Constants.MiscConstants.IMU_ID;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadAnalogDeadbandConfig;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import frc.robot.BreakerLib.util.math.functions.BreakerBezierCurve;
import frc.robot.BreakerLib.util.robot.BreakerRobotConfig;
import frc.robot.BreakerLib.util.robot.BreakerRobotManager;
import frc.robot.BreakerLib.util.robot.BreakerRobotStartConfig;
import frc.robot.commands.BalanceChargingStation;
import frc.robot.commands.autos.GateLeaveThenBalance;
import frc.robot.commands.autos.InNOut;
import frc.robot.commands.autos.LeaveOnly;
import frc.robot.commands.autos.MidBalance;
import frc.robot.commands.autos.MidLeaveThenBalance;
import frc.robot.commands.autos.SubLeaveThenBalance;
import frc.robot.commands.autos.pose.GatePlace2LeaveThenBalance;
import frc.robot.commands.autos.test.TurnTestAuto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.SebArm;

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
  private final SebArm armSys = new SebArm(controllerSys);
  private final RollerIntake intakeSys = new RollerIntake();
  private static boolean isInCubeMode = true;
  // private final Arm armSys = new Arm();
  // private final RollerIntake rollerIntake = new RollerIntake();

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
    SmartDashboard.putBoolean("Is in Cube Mode", isInCubeMode);
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

    controllerSys.getBackButton().onTrue(new InstantCommand(drivetrainSys::resetOdometryRotation));
    //controllerSys.getStartButton().onTrue(new InstantCommand(RobotContainer::toggleisInCubeMode));
    controllerSys.getStartButton().onTrue(new BalanceChargingStation(drivetrainSys, imuSys));
    
    controllerSys.getButtonB().onTrue(new InstantCommand(intakeSys::stop));
    controllerSys.getButtonA().onTrue(new InstantCommand(intakeSys::eject));
    controllerSys.getButtonY().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> armSys.setArmState(SebArm.State.PICKUP_HIGH)),
      new InstantCommand(intakeSys::start)
      ));
    controllerSys.getButtonX().onTrue(new ParallelCommandGroup(
      armSys.pickupLowCommand(),
      new InstantCommand(intakeSys::start)
      ));
    
    controllerSys.getLeftBumper().onTrue(new InstantCommand(intakeSys::start));
    controllerSys.getRightBumper().onTrue(new ParallelCommandGroup(
      armSys.stowCommand(),
      new InstantCommand(intakeSys::stop)
      ));
    
    controllerSys.getDPad().getUp().onTrue(new InstantCommand(armSys::placeMid));
    controllerSys.getDPad().getLeft().onTrue(armSys.setTargetCommand(Rotation2d.fromDegrees(90)));
    // controllerSys.getButtonY().onTrue(new InstantCommand(() -> armSys.setTarget(Rotation2d.fromDegrees(-45))));
    // controllerSys.getButtonX().onTrue(new InstantCommand(() -> armSys.setTarget(Rotation2d.fromDegrees(90))));
    // controllerSys.getButtonA().onTrue(new InstantCommand(() -> armSys.setTarget(Rotation2d.fromDegrees(210))));
    


    // controllerSys.getStartButton().onTrue(new
    // InstantCommand(rollerIntake::runSelectedIntakeMode));
    // controllerSys.getStartButton().onTrue(new
    // InstantCommand(rollerIntake::toggleConeModeSelected));
  }

  private void robotManagerSetup() {
    BreakerRobotConfig robotConfig = new BreakerRobotConfig(new BreakerRobotStartConfig(5104, "BreakerBots",
        "Cthulhu", 2023, "v1", "Yousif Alkhalaf, Roman Abrahamson, Sebastian Rueda"));

    BreakerRobotManager.setup(drivetrainSys, robotConfig);
  }

  public static boolean isInCubeMode() {
    return isInCubeMode;
  }

  public static void toggleisInCubeMode() {
    isInCubeMode = !isInCubeMode;
    SmartDashboard.putBoolean("Is in Cube Mode", isInCubeMode);
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int pathNum = 1;
    switch (pathNum) {
      case 0:
        return new GateLeaveThenBalance(drivetrainSys, imuSys);
      case 1:
        return new MidLeaveThenBalance(drivetrainSys, imuSys, armSys, intakeSys);
      case 2:
        return new SubLeaveThenBalance(drivetrainSys, imuSys);
      case 3:
        return new GatePlace2LeaveThenBalance(drivetrainSys, imuSys, intakeSys, armSys);
      case 4:
        return new InNOut(drivetrainSys, imuSys);
      case 5:
        return new MidBalance(drivetrainSys, imuSys, armSys, intakeSys);
      case 6:
        return new LeaveOnly(drivetrainSys, imuSys);
      case 7:
        return new TurnTestAuto(drivetrainSys, imuSys);
      default:
        return null;

    }
  }
}
