// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveDriveFusedVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveMovementPreferences.SwerveMovementRefrenceFrame;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.suites.BreakerGenericTestSuiteImplementation;
import frc.robot.BreakerLib.util.test.suites.drivetrain.swerve.BreakerSwerveDriveTestSuite;

public class BreakerSwerveDrive extends BreakerGenericDrivetrain implements BreakerGenericTestSuiteImplementation<BreakerSwerveDriveTestSuite> {
  /** Creates a new BreakerSwerveDrive2. */
  /**
   * The current {@link SwerveModuleState} each of this drivetrain's swerve
   * modules is set to
   */
  private SwerveModuleState[] targetModuleStates;

  /**
   * Each of the {@link BreakerGenericSwerveModule} instances controlled by this
   * class.
   */
  private BreakerGenericSwerveModule[] swerveModules;

  /**
   * The {@link BreakerGenericGyro} used for this drivetrain's internal odometery.
   */
  private BreakerGenericGyro gyro;

  /**
   * The {@link BreakerGenericOdometer} object this drivetrain uses for its internal
   * odometry.
   */
  private BreakerGenericOdometer odometer;

  private BreakerSwerveDriveConfig config;

  private SwerveDriveKinematics kinematics;

  private Rotation2d fieldRelativeMovementOffset = new Rotation2d();


  public BreakerSwerveDrive(
    BreakerSwerveDriveConfig config, BreakerGenericGyro gyro, 
    BreakerGenericSwerveModule... swerveModules) {
    this(config, new BreakerSwerveOdometryConfig(), gyro, swerveModules);
  }

  public BreakerSwerveDrive(
    BreakerSwerveDriveConfig config, BreakerSwerveOdometryConfig odometryConfig, 
    BreakerGenericGyro gyro, BreakerGenericSwerveModule... swerveModules) {
    this.config = config;
    this.swerveModules = swerveModules;
    this.gyro = gyro;
    deviceName = "Swerve_Drivetrain";
    targetModuleStates = new SwerveModuleState[swerveModules.length];
    Translation2d[] wheelPositions = new Translation2d[swerveModules.length];
    for (int i = 0; i < targetModuleStates.length; i++) {
      targetModuleStates[i] = new SwerveModuleState();
      wheelPositions[i] = swerveModules[i].getWheelPositionRelativeToRobot();
    }
    kinematics = new SwerveDriveKinematics(wheelPositions);
    odometer = odometryConfig.getOdometer(this);
  }

  /**
   * Directly exposes full module controll to the user, does not proform any
   * automatic optimization or desaturation
   * <p>
   * NOTE: Not affected by slow mode, does not consider technical limits of
   * modules or drivetrain,
   */
  public void setRawModuleStates(SwerveModuleState... targetModuleStates) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setModuleTarget(targetModuleStates[i]);
      this.targetModuleStates[i] = targetModuleStates[i];
    }
  }

  /**
   * Sets each module to match a target module state in the order they were passed
   * in. Automaticly optimizes and desaturates wheelspeeds, stops module if set
   * speed is below module speed deadband threshold
   * <p>
   * NOTE: Not affected by slow mode.
   */
  public void setModuleStates(SwerveModuleState... targetModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, config.getMaxAttainableModuleWheelSpeed());
    for (int i = 0; i < swerveModules.length; i++) {

      if (Math.abs(targetModuleStates[i].speedMetersPerSecond) < config.getModuleWheelSpeedDeadband()) {

        swerveModules[i].stop();

        this.targetModuleStates[i] = swerveModules[i].getModuleTargetState();

      } else {

        SwerveModuleState optimizedState = SwerveModuleState.optimize(targetModuleStates[i],
            Rotation2d.fromDegrees(swerveModules[i].getModuleRelativeAngle()));

        swerveModules[i].setModuleTarget(optimizedState);
        this.targetModuleStates[i] = optimizedState;
      }

    }
  }

  public void move(ChassisSpeeds targetChassisSpeeds, BreakerSwerveMovementPreferences movementPreferences) {
    ChassisSpeeds targetVels = targetChassisSpeeds;
    Rotation2d curAng = odometer.getOdometryPoseMeters().getRotation();

    switch(movementPreferences.getSwerveMovementRefrenceFrame()) {
        case FIELD_RELATIVE_WITHOUT_OFFSET:
            targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds, curAng);
        case FIELD_RELATIVE_WITH_OFFSET:
            targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds,
                curAng.plus(fieldRelativeMovementOffset));
            break;
        case ROBOT_RELATIVE:
        default:
            break;
    }

    if (movementPreferences.getSlowModeValue() == SlowModeValue.ENABLED || (movementPreferences.slowModeValue == SlowModeValue.DEFAULT && slowModeActive)) {
        targetVels.vxMetersPerSecond *= config.getSlowModeLinearMultiplier();
        targetVels.vyMetersPerSecond *= config.getSlowModeLinearMultiplier();
        targetVels.omegaRadiansPerSecond *= config.getSlowModeTurnMultiplier();
    }

    setModuleStates(getKinematics().toSwerveModuleStates(targetChassisSpeeds));
  }

  public void move(ChassisSpeeds targetChassisSpeeds) {
    move(targetChassisSpeeds, new BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT));
  }

  public void move(double percentX, double percentY, double percentOmega, BreakerSwerveMovementPreferences movementPreferences) {
    move(percentsToChassisSpeeds(percentX, percentY, percentOmega), movementPreferences);
  } 

  public void move(double percentX, double percentY, double percentOmega) {
    move(percentX, percentY, percentOmega, new BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT));
  } 

  protected ChassisSpeeds percentsToChassisSpeeds(double percentX, double percentY, double percentOmega) {
    double clampedX = MathUtil.clamp(percentX, -1.0, 1.0);
    double clampedY = MathUtil.clamp(percentY, -1.0, 1.0);
    double clampedOmega = MathUtil.clamp(percentOmega, -1.0, 1.0);
    return new ChassisSpeeds(clampedX * config.getMaxForwardVel(), clampedY  * config.getMaxSidewaysVel(), clampedOmega * config.getMaxAngleVel());
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public BreakerSwerveDriveConfig getConfig() {
      return config;
  }

  /** @return States of swerve modules. */
  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      moduleStates[i] = swerveModules[i].getModuleState();
    }
    return moduleStates;
  }
  
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] moduleStates = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      moduleStates[i] = swerveModules[i].getModulePosition();
    }
    return moduleStates;
  }

  public void setFieldRelativeMovementOffsetAngle(Rotation2d fieldRelativeMovementOffset) {
    this.fieldRelativeMovementOffset = fieldRelativeMovementOffset;
  }

  public Rotation2d getFieldRelativeMovementOffsetAngle() {
    return fieldRelativeMovementOffset;
  }
  
  public void resetSwerveModuleDriveDistances() {
    for (BreakerGenericSwerveModule mod : swerveModules) {
      mod.resetModuleDriveEncoderPosition();
    }
  }

  public void setOdometer(BreakerGenericOdometer odometer) {
      this.odometer = odometer;
  }

  @Override
  public void setOdometryPosition(Pose2d newPose) {
    odometer.setOdometryPosition(newPose);
    
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return odometer.getOdometryPoseMeters();
  }

  @Override
  public BreakerMovementState2d getMovementState() {
    return odometer.getMovementState();
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return odometer.getRobotRelativeChassisSpeeds();
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return odometer.getFieldRelativeChassisSpeeds();
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds(BreakerGenericOdometer odometer) {
    return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(),
        odometer.getOdometryPoseMeters().getRotation());
  }

  @Override
  public void runSelfTest() {
    faultStr = "";
    health = DeviceHealth.NOMINAL;
    for (BreakerGenericSwerveModule module : swerveModules) {
      module.runSelfTest();
      if (module.hasFault()) {
        faultStr += " " + module.getDeviceName() + ": " + module.getFaults() + " ";
        health = health != DeviceHealth.INOPERABLE ? module.getHealth() : health;
      }
    }
  }

  @Override
  public BreakerSwerveDriveTestSuite getTestSuite() {
    return new BreakerSwerveDriveTestSuite(this, swerveModules);
  }


  @Override
  public BreakerGenericGyro getBaseGyro() {
    return gyro;
  }

  @Override
  public void setDrivetrainBrakeMode(boolean isEnabled) {
    for (BreakerGenericSwerveModule module : swerveModules) {
      if (RobotState.isEnabled()) {
        module.setDriveMotorBrakeMode(isEnabled);
        module.setTurnMotorBrakeMode(true);
      } else {
        module.setModuleBrakeMode(isEnabled);
      }
    }
  }

  public SwerveModuleState[] getTargetModuleStates() {
    return targetModuleStates;
  }


  @Override
  public void stop() {
   move(new ChassisSpeeds(), BreakerSwerveMovementPreferences.DEFAULT_ROBOT_RELATIVE_PREFERENCES);
  }

  @Override
  public String toString() {
    return String.format("BreakerSwerveDrive(Health: %s, Movement_State: %s, Swerve_Modules: %s)", health.toString(),
        odometer.getMovementState().toString(), Arrays.toString(swerveModules));
  }

  public static class BreakerSwerveMovementPreferences {
    protected final SwerveMovementRefrenceFrame swerveMovementRefrenceFrame;
    protected final SlowModeValue slowModeValue;
    protected final boolean headingCorrectionEnabled;
    public static final BreakerSwerveMovementPreferences DEFAULT_ROBOT_RELATIVE_PREFERENCES = new BreakerSwerveMovementPreferences().withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame.ROBOT_RELATIVE);
    public static final BreakerSwerveMovementPreferences DEFAULT_FIELD_RELATIVE_PREFERENCES = new BreakerSwerveMovementPreferences();

    /** Uses the drivetrain as odometry provider and uses a field relative movement angle offset. */
    public BreakerSwerveMovementPreferences() {
        this(SwerveMovementRefrenceFrame.ROBOT_RELATIVE, SlowModeValue.DEFAULT, false);
    }

    public BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue) {
        this(movementRefrenceFrame, slowModeValue, false);
    } 

    protected BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, boolean headingCorrectionEnabled) {
      this.slowModeValue = slowModeValue;
      this.swerveMovementRefrenceFrame = movementRefrenceFrame;
      this.headingCorrectionEnabled = headingCorrectionEnabled;
    } 

    public BreakerSwerveMovementPreferences withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame) {
        return new BreakerSwerveMovementPreferences(swerveMovementRefrenceFrame, this.slowModeValue, this.headingCorrectionEnabled);
    }

    /** Sets whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
     * @param slowModeValue Whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
     * @return this.
     */
    public BreakerSwerveMovementPreferences withSlowModeValue(SlowModeValue slowModeValue) {
        return new BreakerSwerveMovementPreferences(this.swerveMovementRefrenceFrame, slowModeValue, this.headingCorrectionEnabled);
    }

    public SlowModeValue getSlowModeValue() {
        return slowModeValue;
    }

    public SwerveMovementRefrenceFrame getSwerveMovementRefrenceFrame() {
        return swerveMovementRefrenceFrame;
    }

    public boolean getHeadingCorrectionEnabled() {
        return headingCorrectionEnabled;
    }

    public enum SwerveMovementRefrenceFrame {
        FIELD_RELATIVE_WITH_OFFSET,
        FIELD_RELATIVE_WITHOUT_OFFSET,
        ROBOT_RELATIVE
    }
}

  public static class BreakerSwerveOdometryConfig {
    private BreakerGenericVisionOdometer vision;
    private Pose2d initalPose;
    private double[] stateStanderdDeveation, visionStanderdDeveation;
    private boolean usePoseEstimator;

    public BreakerSwerveOdometryConfig() {
       this(new Pose2d());
    }

    public BreakerSwerveOdometryConfig(Pose2d initalPose) {
        this.initalPose = initalPose;
        usePoseEstimator = false;
    }

    public BreakerSwerveOdometryConfig(
        BreakerGenericVisionOdometer vision,
        Pose2d initalPose,
        double[] stateStanderdDeveation,
        double[] visionStanderdDeveation
        ) {
       this.initalPose = initalPose;
       this.vision = vision;
       this.stateStanderdDeveation = stateStanderdDeveation;
       this.visionStanderdDeveation = visionStanderdDeveation;
       usePoseEstimator = true;
    }

    public BreakerSwerveOdometryConfig(
        BreakerGenericVisionOdometer vision,
        double[] stateStanderdDeveation,
        double[] visionStanderdDeveation
        ) {
        this(vision, new Pose2d(), stateStanderdDeveation, visionStanderdDeveation);
    }

    public BreakerGenericOdometer getOdometer(BreakerSwerveDrive drivetrain) {
        if (usePoseEstimator) {
            return new BreakerSwerveDriveFusedVisionPoseEstimator(drivetrain, vision, initalPose, visionStanderdDeveation, stateStanderdDeveation);
        }
        return new BreakerSwerveOdometer(drivetrain, initalPose);
    }
}


}
