// Prototyping a new example RobotContainer setup. 

package frc.robot;

import static frc.robot.Constants.MiscConstants.CANIVORE_1;
import static frc.robot.Constants.MiscConstants.IMU_ID;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoManager;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoPath;
import frc.robot.BreakerLib.devices.cosmetic.led.BreakerRevBlinkin;
import frc.robot.BreakerLib.devices.cosmetic.led.BreakerRevBlinkin.AdvancedPattern;
import frc.robot.BreakerLib.devices.cosmetic.led.BreakerRevBlinkin.AdvancedPatternPalette;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadAnalogDeadbandConfig;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.subsystem.cores.arm.BreakerFalconArm;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import frc.robot.BreakerLib.util.math.functions.BreakerBezierCurve;
import frc.robot.BreakerLib.util.robot.BreakerRobotConfig;
import frc.robot.BreakerLib.util.robot.BreakerRobotManager;
import frc.robot.BreakerLib.util.robot.BreakerRobotStartConfig;
import frc.robot.commands.BalanceChargingStation;
// import frc.robot.commands.autos.InNOut;
// import frc.robot.commands.autos.LeaveOnly;
// import frc.robot.commands.autos.MidBalance;
import frc.robot.commands.autos.pathplanner.TestPath;
// import frc.robot.commands.autos.pose.GatePlaceLeaveThenBalance;
// import frc.robot.commands.autos.pose.GatePlace2;
// import frc.robot.commands.autos.pose.MidPlaceLeaveThenBalance;
// import frc.robot.commands.autos.pose.PlaceMidOnly;
// import frc.robot.commands.autos.pose.SubPlaceLeaveThenBalance;
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

  private final BreakerPigeon2 imuSys = new BreakerPigeon2(IMU_ID);
  private final Drive drivetrainSys = new Drive(imuSys);
  private final BreakerBezierCurve driveCurve = new BreakerBezierCurve(new Translation2d(0.707, 0.186),
      new Translation2d(0.799, 0.317));
  private final BreakerTeleopSwerveDriveController manualDriveCommand = new BreakerTeleopSwerveDriveController(
      drivetrainSys, controllerSys).addSpeedCurves(driveCurve, driveCurve);

  private BreakerFalconArm arm = new BreakerFalconArm();

  // private final SebArm armSys = new SebArm(controllerSys);
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
    PathPlannerServer.startServer(5811);
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
    controllerSys.getStartButton().onTrue(new InstantCommand(RobotContainer::toggleisInCubeMode));

    controllerSys.getButtonY().onTrue(new InstantCommand(() -> arm.setGoal(arm.getPosDeg())));

    controllerSys.getButtonB().onTrue(new InstantCommand(intakeSys::stop));
    controllerSys.getButtonA().onTrue(new InstantCommand(intakeSys::eject));
    // controllerSys.getButtonY().onTrue(new ParallelCommandGroup(
    // new InstantCommand(() -> armSys.setArmState(SebArm.State.PICKUP_HIGH)),
    // new InstantCommand(intakeSys::start)));
    // controllerSys.getButtonX().onTrue(new ParallelCommandGroup(
    // armSys.pickupLowCommand(),
    // new InstantCommand(intakeSys::start)));

    controllerSys.getLeftBumper().onTrue(new InstantCommand(intakeSys::start));
    // controllerSys.getRightBumper().onTrue(new ParallelCommandGroup(
    // armSys.stowCommand(),
    // new InstantCommand(intakeSys::stop)));
    // controllerSys.getDPad().getUp().onTrue(new InstantCommand(armSys::placeMid));
    // controllerSys.getDPad().getLeft().onTrue(armSys.setTargetCommand(Rotation2d.fromDegrees(90)));
    // controllerSys.getButtonY().onTrue(new InstantCommand(() ->
    // armSys.setTarget(Rotation2d.fromDegrees(-45))));
    // controllerSys.getButtonX().onTrue(new InstantCommand(() ->
    // armSys.setTarget(Rotation2d.fromDegrees(90))));
    // controllerSys.getButtonA().onTrue(new InstantCommand(() ->
    // armSys.setTarget(Rotation2d.fromDegrees(210))));

    // controllerSys.getStartButton().onTrue(new
    // InstantCommand(rollerIntake::runSelectedIntakeMode));
    // controllerSys.getStartButton().onTrue(new
    // InstantCommand(rollerIntake::toggleConeModeSelected));
  }

  private void robotManagerSetup() {
    BreakerRobotConfig robotConfig = new BreakerRobotConfig(new BreakerRobotStartConfig(5104, "BreakerBots",
        "Cthulhu", 2023, "v1", "Yousif Alkhalaf, Roman Abrahamson, Sebastian Rueda"));

    robotConfig.setAutoPaths(
        // new BreakerAutoPath("GatePlace2", new GatePlace2(drivetrainSys, imuSys,
        // intakeSys, armSys)),
        // new BreakerAutoPath("GatePlaceLeaveThenBalance", new
        // GatePlaceLeaveThenBalance(drivetrainSys, imuSys, armSys, intakeSys)),
        // new BreakerAutoPath("MidPlaceLeaveThenBalance", new
        // MidPlaceLeaveThenBalance(drivetrainSys, imuSys, armSys, intakeSys)),
        // new BreakerAutoPath("SubPlaceLeaveThenBalance", new
        // SubPlaceLeaveThenBalance(drivetrainSys, imuSys, armSys, intakeSys)),
        // new BreakerAutoPath("MidBalance", new MidBalance(drivetrainSys, imuSys,
        // armSys, intakeSys)),
        // new BreakerAutoPath("LeaveOnly", new LeaveOnly(drivetrainSys, imuSys)),
        // new BreakerAutoPath("PlaceLow", new InstantCommand(intakeSys::eject)),
        // new BreakerAutoPath("PlaceMidOnly", new PlaceMidOnly(drivetrainSys, armSys,
        // intakeSys)),
        new BreakerAutoPath("Pathplanner_Test", new TestPath(drivetrainSys)));
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
    return BreakerRobotManager.getSelectedAutoPath();
  }
}
