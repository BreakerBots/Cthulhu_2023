// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.Objects;

import javax.management.loading.PrivateClassLoader;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.auto.pathplanner.BreakerSwervePathFollower;
import frc.robot.BreakerLib.auto.pathplanner.BreakerSwervePathFollower.BreakerSwervePathFollowerConfig;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveDriveFusedVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerGenericVisionOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;

/** Add your docs here. */
public class BreakerSwerveDriveBase extends BreakerSwerveDrive {
    private BreakerSwerveDriveBaseConfig config;
    private BreakerGenericOdometer odometer;
    private BreakerSwervePathFollowerConfig pathFollowerConfig;
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
        pathFollowerConfig = new BreakerSwervePathFollowerConfig(this, odometer, config.getDriveController(), false);
    }

    public void move(ChassisSpeeds targetChassisSpeeds, BreakerSwerveDriveBaseMovementPreferences movementPreferences) {
        
    }

    public void move(ChassisSpeeds targetChassisSpeeds) {
        // move(targetChassisSpeeds, );)
    }

    public BreakerSwervePathFollower followTrajectoryCommand(PathPlannerTrajectory trajectory) {
        return new BreakerSwervePathFollower(pathFollowerConfig, trajectory, slowModeActive);
    }

    @Override
    public BreakerSwerveDriveBaseConfig getConfig() {
        return config;
    }

    public static class BreakerSwerveDriveBaseMovementPreferences {
        private SwerveMovementRefrenceFrameValue swerveMovementRefrenceFrameValue;
        private SlowModeValue slowModeValue;
        private boolean headingCorrectionEnabled;
        public static final BreakerSwerveDriveBaseMovementPreferences DEFAULT_ROBOT_RELATIVE_PREFERENCES = new BreakerSwerveDriveBaseMovementPreferences();
        public static final BreakerSwerveDriveBaseMovementPreferences DEFAULT_FIELD_RELATIVE_PREFERENCES = new BreakerSwerveDriveBaseMovementPreferences().withSwerveMovementRefrenceFrameValue(SwerveMovementRefrenceFrameValue.ROBOT_RELATIVE);
    
        /** Uses the drivetrain as odometry provider and uses a field relative movement angle offset. */
        public BreakerSwerveDriveBaseMovementPreferences() {
            swerveMovementRefrenceFrameValue = SwerveMovementRefrenceFrameValue.FIELD_RELATIVE_WITH_OFFSET;
            slowModeValue = SlowModeValue.DEFAULT;
            headingCorrectionEnabled = true;
        }
    
        public BreakerSwerveDriveBaseMovementPreferences(SwerveMovementRefrenceFrameValue movementRefrenceFrameValue, SlowModeValue slowModeValue, boolean headingCorrectionEnabled) {
          this.slowModeValue = slowModeValue;
          this.swerveMovementRefrenceFrameValue = movementRefrenceFrameValue;
          this.headingCorrectionEnabled = headingCorrectionEnabled;
        }
    
        public BreakerSwerveDriveBaseMovementPreferences withSwerveMovementRefrenceFrameValue(SwerveMovementRefrenceFrameValue swerveMovementRefrenceFrameValue) {
            this.swerveMovementRefrenceFrameValue = swerveMovementRefrenceFrameValue;
            return this;
        }
    
        /** Sets whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
         * @param slowModeValue Whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
         * @return this.
         */
        public BreakerSwerveDriveBaseMovementPreferences withSlowModeValue(SlowModeValue slowModeValue) {
            this.slowModeValue = slowModeValue;
            return this;
        }

        public BreakerSwerveDriveBaseMovementPreferences withHeadingCorrectionEnabled(boolean headingCorrectionEnabled) {
            this.headingCorrectionEnabled = headingCorrectionEnabled;
            return this;
        }
    
        public SlowModeValue getSlowModeValue() {
            return slowModeValue;
        }

        public SwerveMovementRefrenceFrameValue getMovementRefrenceFrameValue() {
            return swerveMovementRefrenceFrameValue;
        }

        public boolean isHeadingCorrectionEnabled() {
            return headingCorrectionEnabled;
        }

        public enum SwerveMovementRefrenceFrameValue {
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
