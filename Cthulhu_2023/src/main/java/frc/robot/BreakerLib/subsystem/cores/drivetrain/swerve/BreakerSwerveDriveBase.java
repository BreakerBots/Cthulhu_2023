// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.HashMap;
import java.util.Objects;

import javax.management.loading.PrivateClassLoader;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.auto.pathplanner.BreakerSwervePathFollower;
import frc.robot.BreakerLib.auto.pathplanner.BreakerSwervePathFollower.BreakerSwervePathFollowerConfig;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveDriveFusedVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase.BreakerSwerveDriveBaseMovementPreferences.SwerveMovementRefrenceFrame;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase.BreakerSwerveDriveBaseMovementPreferences;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;

/** Add your docs here. */
public class BreakerSwerveDriveBase extends BreakerSwerveDrive {
    private BreakerSwerveDriveBaseConfig config;
    private BreakerGenericOdometer odometer;
    private BreakerSwervePathFollowerConfig pathFollowerConfig;
    private Rotation2d lastSetHeading;
    public BreakerSwerveDriveBase(
            BreakerSwerveDriveBaseConfig config, BreakerGenericGyro gyro,
            BreakerGenericSwerveModule... swerveModules) {
        this(config, gyro, new BreakerSwerveDriveBaseOdometryConfig(), swerveModules);
    }

    public BreakerSwerveDriveBase(
            BreakerSwerveDriveBaseConfig config, BreakerGenericGyro gyro, 
            BreakerSwerveDriveBaseOdometryConfig odometryConfig,
            BreakerGenericSwerveModule... swerveModules) {
        super(config, gyro, swerveModules);
        this.config = config;
        odometer = odometryConfig.getOdometer(this);
        lastSetHeading = odometer.getOdometryPoseMeters().getRotation();
        pathFollowerConfig = new BreakerSwervePathFollowerConfig(this, odometer, config.getDriveController(), false);
    }

    public void drive(ChassisSpeeds targetChassisSpeeds, BreakerSwerveDriveBaseMovementPreferences movementPreferences) {
        ChassisSpeeds targetVels = targetChassisSpeeds;
        Rotation2d curAng = odometer.getOdometryPoseMeters().getRotation();

        switch(movementPreferences.getSwerveMovementRefrenceFrame()) {
            case FIELD_RELATIVE_WITHOUT_OFFSET:
                targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds, curAng);
            case FIELD_RELATIVE_WITH_OFFSET:
                targetVels = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds,
                    curAng.plus(getFieldRelativeMovementOffsetAngle()));
                break;
            case ROBOT_RELATIVE:
            default:
                break;
        }

        if (Math.abs(targetChassisSpeeds.omegaRadiansPerSecond) < config.getAngularVelDeadband()) {
            if (movementPreferences.getHeadingCorrectionEnabled()) {
                targetVels.omegaRadiansPerSecond = config.thetaController.calculate(curAng.getRadians(), lastSetHeading.getRadians());
            }
        } else {
            lastSetHeading = curAng;
        }

        if (movementPreferences.getSlowModeValue() == SlowModeValue.ENABLED || (movementPreferences.slowModeValue == SlowModeValue.DEFAULT && slowModeActive)) {
            targetVels.vxMetersPerSecond *= config.getSlowModeLinearMultiplier();
            targetVels.vyMetersPerSecond *= config.getSlowModeLinearMultiplier();
            targetVels.omegaRadiansPerSecond *= config.getSlowModeTurnMultiplier();
        }

        setModuleStates(getKinematics().toSwerveModuleStates(targetChassisSpeeds));
    }

    public void drive(ChassisSpeeds targetChassisSpeeds) {
        drive(targetChassisSpeeds, BreakerSwerveDriveBaseMovementPreferences.DEFAULT_FIELD_RELATIVE_PREFERENCES);
    }


    public void drive(double percentX, double percentY, double precentOmega, BreakerSwerveDriveBaseMovementPreferences movementPreferences) {
        drive(new ChassisSpeeds(percentX * config.getMaxForwardVel(), percentY * config.getMaxSidewaysVel(), precentOmega * config.getMaxAngleVel()), movementPreferences);
    } 
    
    public void drive(double percentX, double percentY, double precentOmega) {
        drive(new ChassisSpeeds(percentX * config.getMaxForwardVel(), percentY * config.getMaxSidewaysVel(), precentOmega * config.getMaxAngleVel()), BreakerSwerveDriveBaseMovementPreferences.DEFAULT_FIELD_RELATIVE_PREFERENCES);
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
   * @param slowModeValue           Whether or not to apply the set slow mode
   *                                multiplier to the given speeds or to use global default.
   */
  public void move(ChassisSpeeds robotRelativeVelocities, SlowModeValue slowModeValue) {
    drive(robotRelativeVelocities, BreakerSwerveDriveBaseMovementPreferences.DEFAULT_ROBOT_RELATIVE_PREFERENCES.withSlowModeValue(slowModeValue));
  }

  /** Sets the target velocity of the robot to 0 in all axes. */
  public void stop() {
    drive(new ChassisSpeeds(), BreakerSwerveDriveBaseMovementPreferences.DEFAULT_ROBOT_RELATIVE_PREFERENCES.withHeadingCorrectionEnabled(false));
  }

    /**
   * Movement with speeds passed in as a percentage.
   * 
   * @param forwardPercent    Forward speed % (-1 to 1).
   * @param horizontalPercent Sideways speed % (-1 to 1).
   * @param turnPercent       Turn speed % (-1 to 1).
   * @param slowModeValue     Weather or not to use slow mode or default to global setting
   */
  public void moveWithPercentInput(double forwardPercent, double horizontalPercent, double turnPercent, SlowModeValue slowModeValue) {
    drive(forwardPercent, horizontalPercent, turnPercent, BreakerSwerveDriveBaseMovementPreferences.DEFAULT_ROBOT_RELATIVE_PREFERENCES.withSlowModeValue(slowModeValue));
  }

  /**k
   * Movement with velocities relative to field. Use if using a separate odometry
   * source.
   * 
   * @param forwardVelMetersPerSec              Forward velocity relative to field
   *                                            (m/s).
   * @param preferences Field movement preferences to use.
   */
  public void moveRelativeToField(ChassisSpeeds fieldRelativeSpeeds,
      BreakerSwerveFieldRelativeMovementPreferences preferences) {
    drive(fieldRelativeSpeeds, new BreakerSwerveDriveBaseMovementPreferences(preferences));
  }

  /**
   * Movement with velocity values relative to field.
   * 
   * @param fieldRelativeSpeeds Field relative speeds to use.
   */
  public void moveRelativeToField(ChassisSpeeds fieldRelativeSpeeds) {
    drive(fieldRelativeSpeeds, BreakerSwerveDriveBaseMovementPreferences.DEFAULT_FIELD_RELATIVE_PREFERENCES);
  }

  /**
   * Movement with velocities relative to field. Use if using a separate odometry
   * source.
   * 
   * @param forwardVelMetersPerSec              Forward velocity relative to field
   *                                            (m/s).
   * @param horizontalVelMetersPerSec           Sideways velocity relative to
   *                                            field (m/s).
   * @param radPerSec                           Rotation velocity relative to
   *                                            field (rad/sec).
   * @param odometer                            {@link BreakerGenericOdometer} to
   *                                            determine field relative position.
   * @param useFieldRelativeMovementAngleOffset weather or not to use the set
   *                                            angle offset for the field
   *                                            relative mobement forward angle
   *                                            zero point
   */
  public void moveRelativeToField(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec,
      BreakerSwerveFieldRelativeMovementPreferences prefrences) {
    drive(new ChassisSpeeds(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec), new BreakerSwerveDriveBaseMovementPreferences(prefrences));
  }

  /**
   * Movement with velocity percents relative to field. Swerve drive's own
   * odometry is used.
   * 
   * @param forwardPercent    Forward speed percent (-1 to 1).
   * @param horizontalPercent Horizontal speed percent (-1 to 1).
   * @param turnPercent       Rotation speed percent (-1 to 1).
   */
  public void moveWithPercentInputRelativeToField(double forwardPercent, double horizontalPercent, double turnPercent) {
    drive(forwardPercent, horizontalPercent, turnPercent, BreakerSwerveDriveBaseMovementPreferences.DEFAULT_FIELD_RELATIVE_PREFERENCES);
  }

  /**
   * Movement with velocity percents relative to field. Use if using a separate
   * odometry source.
   * 
   * @param forwardPercent                      Forward speed percent (-1 to 1).
   * @param horizontalPercent                   Horizontal speed percent (-1 to
   *                                            1).
   * @param turnPercent                         Rotation speed percent (-1 to 1).
   * @param odometer                            {@link BreakerGenericOdometer} to
   *                                            determine field relative position.
   * @param useFieldRelativeMovementAngleOffset weather or not to use the set
   *                                            angle offset for the field
   *                                            relative mobement forward angle
   *                                            zero point
   */
  public void moveWithPercentInputRelativeToField(double forwardPercent, double horizontalPercent, double turnPercent,
      BreakerSwerveFieldRelativeMovementPreferences prefrences) {
        drive(forwardPercent, horizontalPercent, turnPercent, new BreakerSwerveDriveBaseMovementPreferences(prefrences));
  }

    public BreakerSwervePathFollower followPathCommand(PathPlannerTrajectory path) {
        return new BreakerSwervePathFollower(pathFollowerConfig, path, slowModeActive);
    }

    public FollowPathWithEvents followPathWithEventsCommand(PathPlannerTrajectory path, HashMap<String, Command> eventMap) {
        return new FollowPathWithEvents(followPathCommand(path), path.getMarkers(), eventMap);
    }

    @Override
    public BreakerSwerveDriveBaseConfig getConfig() {
        return config;
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        odometer.setOdometryPosition(newPose);
        lastSetHeading = newPose.getRotation();
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return super.getOdometryPoseMeters();
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

    public static class BreakerSwerveDriveBaseMovementPreferences {
        private final SwerveMovementRefrenceFrame swerveMovementRefrenceFrame;
        private final SlowModeValue slowModeValue;
        private final boolean headingCorrectionEnabled;
        public static final BreakerSwerveDriveBaseMovementPreferences DEFAULT_ROBOT_RELATIVE_PREFERENCES = new BreakerSwerveDriveBaseMovementPreferences().withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame.ROBOT_RELATIVE);
        public static final BreakerSwerveDriveBaseMovementPreferences DEFAULT_FIELD_RELATIVE_PREFERENCES = new BreakerSwerveDriveBaseMovementPreferences();
    
        /** Uses the drivetrain as odometry provider and uses a field relative movement angle offset. */
        public BreakerSwerveDriveBaseMovementPreferences() {
            swerveMovementRefrenceFrame = SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET;
            slowModeValue = SlowModeValue.DEFAULT;
            headingCorrectionEnabled = true;
        }

        public BreakerSwerveDriveBaseMovementPreferences(BreakerSwerveFieldRelativeMovementPreferences legacyPreferences) {
            this.slowModeValue = legacyPreferences.getSlowModeValue();
            this.swerveMovementRefrenceFrame = legacyPreferences.getUseFieldRelativeMovementAngleOffset() ? SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET : SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET;
            this.headingCorrectionEnabled = false;
          }
    
        public BreakerSwerveDriveBaseMovementPreferences(SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, boolean headingCorrectionEnabled) {
          this.slowModeValue = slowModeValue;
          this.swerveMovementRefrenceFrame = movementRefrenceFrame;
          this.headingCorrectionEnabled = headingCorrectionEnabled;
        }
    
        public BreakerSwerveDriveBaseMovementPreferences withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame) {
            return new BreakerSwerveDriveBaseMovementPreferences(swerveMovementRefrenceFrame, this.slowModeValue, this.headingCorrectionEnabled);
        }
    
        /** Sets whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
         * @param slowModeValue Whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
         * @return this.
         */
        public BreakerSwerveDriveBaseMovementPreferences withSlowModeValue(SlowModeValue slowModeValue) {
            return new BreakerSwerveDriveBaseMovementPreferences(this.swerveMovementRefrenceFrame, slowModeValue, this.headingCorrectionEnabled);
        }

        public BreakerSwerveDriveBaseMovementPreferences withHeadingCorrectionEnabled(boolean headingCorrectionEnabled) {
            return new BreakerSwerveDriveBaseMovementPreferences(this.swerveMovementRefrenceFrame, this.slowModeValue, headingCorrectionEnabled);
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

    public static class BreakerSwerveDriveBaseOdometryConfig {
        private BreakerGenericVisionOdometer vision;
        private Pose2d initalPose;
        private double[] stateStanderdDeveation, visionStanderdDeveation;
        private boolean usePoseEstimator;

        public BreakerSwerveDriveBaseOdometryConfig() {
           this(new Pose2d());
        }

        public BreakerSwerveDriveBaseOdometryConfig(Pose2d initalPose) {
            this.initalPose = initalPose;
            usePoseEstimator = false;
        }

        public BreakerSwerveDriveBaseOdometryConfig(
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

        public BreakerSwerveDriveBaseOdometryConfig(
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
            return drivetrain;
        }
    }

    public static class BreakerSwerveDriveBaseConfig extends BreakerSwerveDriveConfig {
        private PIDController xController, yController, thetaController;
        private PPHolonomicDriveController driveController;
        private double angularVelDeadband;
        public BreakerSwerveDriveBaseConfig(double maxForwardVel, double maxSidewaysVel, double maxAngVel, 
                double angularVelDeadband, double moduleWheelSpeedDeadband, double maxAttainableModuleWheelSpeed,
                PIDController xController, PIDController yController, PIDController thetaController) {
            super(maxForwardVel, maxSidewaysVel, maxAngVel, moduleWheelSpeedDeadband, maxAttainableModuleWheelSpeed);
            this.xController = xController;
            this.yController = yController;
            this.thetaController = thetaController;
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
            this.angularVelDeadband = angularVelDeadband;
            driveController = new PPHolonomicDriveController(xController, yController, thetaController);
        }

        public PIDController getPIDControllerX() {
            return xController;
        }

        public PIDController getPIDControllerY() {
            return yController;
        }

        public PIDController getPIDControllerTheta() {
            return thetaController;
        }

        public PPHolonomicDriveController getDriveController() {
            return driveController;
        }

        public double getAngularVelDeadband() {
            return angularVelDeadband;
        }

    }

}
