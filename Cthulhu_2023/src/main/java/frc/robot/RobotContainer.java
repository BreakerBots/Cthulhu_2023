// Prototyping a new example RobotContainer setup. 

package frc.robot;

import static frc.robot.Constants.MiscConstants.CANIVORE_2;
import static frc.robot.Constants.MiscConstants.IMU_ID;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadAnalogDeadbandConfig;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import frc.robot.BreakerLib.util.math.functions.BreakerBezierCurve;
import frc.robot.BreakerLib.util.robot.BreakerRobotConfig;
import frc.robot.BreakerLib.util.robot.BreakerRobotManager;
import frc.robot.BreakerLib.util.robot.BreakerRobotStartConfig;
import frc.robot.commands.BalanceChargingStation;
import frc.robot.commands.MoveToGamePiece;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPose;
import frc.robot.subsystems.gamepiece.GamePieceTracker;

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
  private final Arm armSys = new Arm();
  private final Gripper gripperSys = new Gripper();
 // private final Odometer odometerSys = new Odometer(drivetrainSys, new BreakerVisionPoseFilter(5.0, 0.35, Constants.Vision.AprilTag.APRILTAGS));
  private final BreakerBezierCurve driveCurve = new BreakerBezierCurve(new Translation2d(0.707, 0.186), new Translation2d(0.799, 0.317));
  private final BreakerTeleopSwerveDriveController manualDriveCommand = new BreakerTeleopSwerveDriveController(
      drivetrainSys, controllerSys).addSpeedCurves(driveCurve, driveCurve);
  //private final AprilTagTracker att = new AprilTagTracker();
  private final GamePieceTracker gpt = new GamePieceTracker();

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
    controllerSys.getButtonB().onTrue(new InstantCommand(drivetrainSys::toggleSlowMode));
    controllerSys.getButtonX().onTrue(new InstantCommand(drivetrainSys::resetOdometryRotation));
    controllerSys.getButtonA().onTrue(new MoveToGamePiece(drivetrainSys, gpt));
    //controllerSys.getButtonY().onTrue(new BalanceChargingStation(drivetrainSys, imuSys));
    controllerSys.getButtonY().onTrue(new InstantCommand(() -> armSys.setManualTargetPose(new ArmPose(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0)))));
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
    //drivetrainSys.getTestSuite().setLogType(BreakerTestSuiteDataLogType.LIVE_AUTOLOG);
    ArrayList<Pair<ChassisSpeeds, Double>> speedList = new ArrayList<>();
    // speedList.add(new Pair<ChassisSpeeds, Double>(new ChassisSpeeds(0, 0, 0.3), 3.0));
    speedList.add(new Pair<ChassisSpeeds, Double>(new ChassisSpeeds(2.0, 0, 0.0), 3.0));
    // speedList.add(new Pair<ChassisSpeeds, Double>(new ChassisSpeeds(0, 0, Math.PI), 3.0));
    // speedList.add(new Pair<ChassisSpeeds, Double>(new ChassisSpeeds(0, 3, 0), 3.0));

   //return new ApriltagTestPath(drivetrainSys, att, imuSys);
   //return drivetrainSys.getTestSuite().stressTest(speedList);
   //return new Pickup1_Place2_Balence_6_3(drivetrainSys, att, imuSys);
   return null;
  }
}
