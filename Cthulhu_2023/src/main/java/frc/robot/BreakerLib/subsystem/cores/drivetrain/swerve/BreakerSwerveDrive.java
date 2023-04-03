// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.suites.BreakerGenericTestSuiteImplementation;
import frc.robot.BreakerLib.util.test.suites.drivetrain.swerve.BreakerSwerveDriveTestSuite;

/**
 * BreakerLib swerve drive class.
 */
public class BreakerSwerveDrive extends BreakerGenericDrivetrain implements BreakerGenericTestSuiteImplementation<BreakerSwerveDriveTestSuite> {
  private BreakerSwerveDriveConfig config;

  /** The current {@link SwerveModuleState} each of this drivetrain's swerve modules is set to */
  private SwerveModuleState[] targetModuleStates;

  /** Each of the {@link BreakerGenericSwerveModule} instances controlled by this class. */
  private BreakerGenericSwerveModule[] swerveModules;

  /** The {@link BreakerGenericGyro} used for this drivetrain's internal odometery. */
  private BreakerGenericGyro gyro;

  /** The {@link SwerveDriveOdometry} object this drivetrain uses for its internal odometry. */
  private SwerveDriveOdometry odometer;

  private BreakerMovementState2d prevMovementState = new BreakerMovementState2d(),
      curMovementState = new BreakerMovementState2d();
  private double prevOdometryUpdateTimestamp = 0;

  private Rotation2d fieldRelativeMovementOffset = new Rotation2d();

  /**
   * Constructs a new swerve based drivetrain.
   * 
   * @param config        The configuration values for the drivetrain's
   *                      characteristics and behavior, passed in as a
   *                      "BreakerSwerveDriveConfig" object
   * @param swerveModules The swerve drive modules that make up the
   *                      drivetrain. Should be passed in the same oreder as the
   *                      translations in your {@link BreakerSwerveDriveConfig}
   */
  public BreakerSwerveDrive(BreakerSwerveDriveConfig config, BreakerGenericGyro gyro,
      BreakerGenericSwerveModule... swerveModules) {
    this.config = config;
    this.swerveModules = swerveModules;
    this.gyro = gyro;
    deviceName = "Swerve_Drivetrain";
    targetModuleStates = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < targetModuleStates.length; i++) {
      targetModuleStates[i] = new SwerveModuleState();
    }
    odometer = new SwerveDriveOdometry(config.getKinematics(), Rotation2d.fromDegrees(gyro.getRawYaw()), getSwerveModulePositions());
  }

  /**
   * Directly exposes full module controll to the user, does not proform any automatic optimization or desaturation
   * <p>
   * NOTE: Not affected by slow mode, does not consider technical limits of modules or drivetrain,
   */
  public void setRawModuleStates(SwerveModuleState... targetModuleStates) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setModuleTarget(targetModuleStates[i]);
      this.targetModuleStates[i] = targetModuleStates[i];
    }
  }

  /**
   * Sets each module to match a target module state in the order they were passed
   * in. Automaticly optimizes and desaturates wheelspeeds, stops module if set speed is below module speed deadband threshold
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

  /**
   * Standard drivetrain movement command. Specifies robot velocity in each axis
   * including robot rotation (rad/sec).
   * <p>
   * NOTE: All values are relative to the robot's orientation.
   * 
   * @param robotRelativeVelocities ChassisSpeeds object representing the robots
   *                                velocities in each axis relative to its local
   *                                reference frame.
   * @param useSlowMode             Whether or not to apply the set slow mode
   *                                multiplier to the given speeds.
   */
  public void move(ChassisSpeeds robotRelativeVelocities, boolean useSlowMode) {
    if (useSlowMode) {
      robotRelativeVelocities.vxMetersPerSecond *= config.getSlowModeLinearMultiplier();
      robotRelativeVelocities.vyMetersPerSecond *= config.getSlowModeLinearMultiplier();
      robotRelativeVelocities.omegaRadiansPerSecond *= config.getSlowModeTurnMultiplier();
    }
    setModuleStates(config.getKinematics().toSwerveModuleStates(robotRelativeVelocities));
  }

  /**
   * Standard drivetrain movement command, specifies robot velocity in each axis
   * including robot rotation (rad/sec). Slow mode is active based on whener or
   * not the user has globaly enabled slow mode.
   * <p>
   * NOTE: All values are relative to the robot's orientation.
   * 
   * @param robotRelativeVelocities ChassisSpeeds object representing the robots
   *                                velocities in each axis relative to its local
   *                                reference frame.
   */
  public void move(ChassisSpeeds robotRelativeVelocities) {
    move(robotRelativeVelocities, slowModeActive);
  }

  /**
   * Standard drivetrain movement command, specifies robot velocity in each axis
   * including robot rotation (radian per second).
   * <p>
   * NOTE: All values are relative to the robot's orientation.
   * 
   * @param forwardVelMetersPerSec    Forward movement velocity in m/s.
   * @param horizontalVelMetersPerSec Sideways movement velocity in m/s.
   * @param radPerSec                 Rotation speed in rad/sec.
   */
  public void move(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec) {
    move(new ChassisSpeeds(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec));
  }

  /**
   * Standard drivetrain movement command, specifies robot velocity in each axis
   * including robot rotation (radian per second).
   * <p>
   * NOTE: All values are relative to the robot's orientation.
   * 
   * @param forwardVelMetersPerSec    Forward movement velocity in m/s.
   * @param horizontalVelMetersPerSec Sideways movement velocity in m/s.
   * @param radPerSec                 Rotation speed in rad/sec.
   * @param useSlowMode               Whether slow mode will be applied or not.
   */
  public void move(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec,
      boolean useSlowMode) {
    move(new ChassisSpeeds(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec), useSlowMode);
  }

  /** Sets the target velocity of the robot to 0 in all axes. */
  public void stop() {
    move(0, 0, 0);
  }

  /**
   * Movement with speeds passed in as a percentage.
   * 
   * @param forwardPercent    Forward speed % (-1 to 1).
   * @param horizontalPercent Sideways speed % (-1 to 1).
   * @param turnPercent       Turn speed % (-1 to 1).
   */
  public void moveWithPercentInput(double forwardPercent, double horizontalPercent, double turnPercent) {
    move(
      (forwardPercent * config.getMaxForwardVel()),
      (horizontalPercent * config.getMaxSidewaysVel()),
      (turnPercent * config.getMaxAngleVel())
    );
  }

    /**
   * Movement with velocities relative to field. Use if using a separate odometry source.
   * 
   * @param forwardVelMetersPerSec Forward velocity relative to field (m/s).
   * @param horizontalVelMetersPerSec Sideways velocity relative to field (m/s).
   * @param radPerSec Rotation velocity relative to field (rad/sec).
   * @param odometer {@link BreakerGenericOdometer} to determine field relative position.
   * @param useFieldRelativeMovementAngleOffset weather or not to use the set angle offset for the field relative mobement forward angle zero point
   */
  public void moveRelativeToField(ChassisSpeeds fieldRelativeSpeeds, BreakerSwerveFieldRelativeMovementPrefrences prefrences) {
    ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, prefrences.odometryProvider.getOdometryPoseMeters().getRotation().plus(prefrences.useFieldRelativeMovementAngleOffset ? fieldRelativeMovementOffset : new Rotation2d()));
    move(robotRelSpeeds);
  }

  /**
   * Movement with velocity values relative to field.
   * 
   * @param forwardVelMetersPerSec Forward velocity relative to field (m/s).
   * @param horizontalVelMetersPerSec Sideways velocity relative to field (m/s).
   * @param radPerSec Rotation velocity relative to field (rad/sec).
   */
  public void moveRelativeToField(ChassisSpeeds fieldRelativeSpeeds) {
    moveRelativeToField(fieldRelativeSpeeds, new BreakerSwerveFieldRelativeMovementPrefrences(this));
  }

  /**
   * Movement with velocities relative to field. Use if using a separate odometry source.
   * 
   * @param forwardVelMetersPerSec Forward velocity relative to field (m/s).
   * @param horizontalVelMetersPerSec Sideways velocity relative to field (m/s).
   * @param radPerSec Rotation velocity relative to field (rad/sec).
   * @param odometer {@link BreakerGenericOdometer} to determine field relative position.
   * @param useFieldRelativeMovementAngleOffset weather or not to use the set angle offset for the field relative mobement forward angle zero point
   */
  public void moveRelativeToField(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec,
    BreakerSwerveFieldRelativeMovementPrefrences prefrences) {
    ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        forwardVelMetersPerSec,
        horizontalVelMetersPerSec, 
        radPerSec,
        prefrences.odometryProvider.getOdometryPoseMeters().getRotation().plus(prefrences.useFieldRelativeMovementAngleOffset ? fieldRelativeMovementOffset : new Rotation2d()));
    move(robotRelSpeeds);
  }

  /**
   * Movement with velocity values relative to field.
   * 
   * @param forwardVelMetersPerSec Forward velocity relative to field (m/s).
   * @param horizontalVelMetersPerSec Sideways velocity relative to field (m/s).
   * @param radPerSec Rotation velocity relative to field (rad/sec).
   */
  public void moveRelativeToField(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec) {
    moveRelativeToField(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec, new BreakerSwerveFieldRelativeMovementPrefrences(this));
  }

  /**
   * Movement with velocity percents relative to field. Swerve drive's own odometry is used.
   * 
   * @param forwardPercent Forward speed percent (-1 to 1).
   * @param horizontalPercent Horizontal speed percent (-1 to 1).
   * @param turnPercent Rotation speed percent (-1 to 1).
   */
  public void moveWithPercentInputRelativeToField(double forwardPercent, double horizontalPercent, double turnPercent) {
    double fwdV = forwardPercent * config.getMaxForwardVel();
    double horzV = horizontalPercent * config.getMaxSidewaysVel();
    double thetaV = turnPercent * config.getMaxAngleVel();
    moveRelativeToField(fwdV, horzV, thetaV, new BreakerSwerveFieldRelativeMovementPrefrences(this));
  }

  /**
   * Movement with velocity percents relative to field. Use if using a separate odometry source.
   * 
   * @param forwardPercent Forward speed percent (-1 to 1).
   * @param horizontalPercent Horizontal speed percent (-1 to 1).
   * @param turnPercent Rotation speed percent (-1 to 1).
   * @param odometer {@link BreakerGenericOdometer} to determine field relative position.
   * @param useFieldRelativeMovementAngleOffset weather or not to use the set angle offset for the field relative mobement forward angle zero point
   */
  public void moveWithPercentInputRelativeToField(double forwardPercent, double horizontalPercent, double turnPercent,
      BreakerSwerveFieldRelativeMovementPrefrences prefrences) {
    double fwdV = forwardPercent * config.getMaxForwardVel();
    double horzV = horizontalPercent * config.getMaxSidewaysVel();
    double thetaV = turnPercent * config.getMaxAngleVel();
    moveRelativeToField(fwdV, horzV, thetaV, prefrences);
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

  public void resetSwerveModuleDriveDistances() {
    for (BreakerGenericSwerveModule mod: swerveModules) {
      mod.resetModuleDriveEncoderPosition();
    }
  }

  @Override
  public void updateOdometry() {
    odometer.update(Rotation2d.fromDegrees(gyro.getRawYaw()), getSwerveModulePositions());
    calculateMovementState((Timer.getFPGATimestamp() - prevOdometryUpdateTimestamp) * 1000);
    prevOdometryUpdateTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return odometer.getPoseMeters();
  }

  public BreakerSwerveDriveConfig getConfig() {
    return config;
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

  @Override
  public BreakerGenericGyro getBaseGyro() {
      return gyro;
  }

  @Override
  public void setOdometryPosition(Pose2d newPose) {
    odometer.resetPosition(Rotation2d.fromDegrees(gyro.getRawYaw()), getSwerveModulePositions(), newPose);
  }

  @Override
  public BreakerMovementState2d getMovementState() {
    return curMovementState;
  }

  private void calculateMovementState(double timeToLastUpdateMiliseconds) {
    // ChassisSpeeds speeds = config.getKinematics().toChassisSpeeds(getSwerveModuleStates());
    curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(),
        getFieldRelativeChassisSpeeds(), timeToLastUpdateMiliseconds, prevMovementState);
    prevMovementState = curMovementState;
  }

  public SwerveModuleState[] getTargetModuleStates() {
    return targetModuleStates;
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return config.getKinematics().toChassisSpeeds(getSwerveModuleStates());
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), getOdometryPoseMeters().getRotation());
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds(BreakerGenericOdometer odometer) {
    return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(),
        odometer.getOdometryPoseMeters().getRotation());
  }

  public void setFieldRelativeMovementOffsetAngle(Rotation2d fieldRelativeMovementOffset) {
    this.fieldRelativeMovementOffset = fieldRelativeMovementOffset;
  }

  public Rotation2d getFieldRelativeMovementOffsetAngle() {
      return fieldRelativeMovementOffset;
  }
  
  @Override
  public String toString() {
      return String.format("BreakerSwerveDrive(Health: %s, Movement_State: %s, Swerve_Modules: %s)", health.toString(), curMovementState.toString(), Arrays.toString(swerveModules));
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  @Override
  public BreakerSwerveDriveTestSuite getTestSuite() {
    return new BreakerSwerveDriveTestSuite(this, swerveModules);
  }

  public static class BreakerSwerveFieldRelativeMovementPrefrences {
    public boolean useFieldRelativeMovementAngleOffset;
    public BreakerGenericOdometer odometryProvider;
    public BreakerSwerveFieldRelativeMovementPrefrences(BreakerSwerveDrive drivetrain) {
      odometryProvider = drivetrain;
      useFieldRelativeMovementAngleOffset = true;
    }

    public BreakerSwerveFieldRelativeMovementPrefrences setOdometryProvider(BreakerGenericOdometer odometryProvider) {
        this.odometryProvider = odometryProvider;
        return this;
    }

    public BreakerSwerveFieldRelativeMovementPrefrences setUseFieldRelativeMovementAngleOffset(boolean useFieldRelativeMovementAngleOffset) {
        this.useFieldRelativeMovementAngleOffset = useFieldRelativeMovementAngleOffset;
        return this;
    }
  }

}
