// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BreakerLib.auto.waypoint.BreakerPoseWaypointPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollowerConfig;
import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerLegacyPigeon2;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModuleBuilder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModuleBuilder.BreakerSwerveModuleConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveLegacyCANcoder;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.MiscConstants.*;

/** Add your docs here. */
public class Drive extends BreakerSwerveDriveBase {

        public static ProfiledPIDController autoAnglePID = new ProfiledPIDController(6.9, 0.0, 0.0,
                        new TrapezoidProfile.Constraints(3.0, 3.0));
        public static PIDController autoDrivePID = new PIDController(6.0, 0.0, 0.0);
        public static BreakerHolonomicDriveController autoDriveController = new BreakerHolonomicDriveController(autoDrivePID, autoAnglePID);
        public BreakerSwerveWaypointFollowerConfig autoConfig = new BreakerSwerveWaypointFollowerConfig(this, autoDriveController);

        private static WPI_TalonFX driveFL = new WPI_TalonFX(FL_DRIVE_ID, CANIVORE_1);
        private static WPI_TalonFX turnFL = new WPI_TalonFX(FL_TURN_ID, CANIVORE_1);
        private static BreakerSwerveAzimuthEncoder encoderFL = new BreakerSwerveLegacyCANcoder(new WPI_CANCoder(FL_ENCODER_ID, CANIVORE_1));

        private static WPI_TalonFX driveFR = new WPI_TalonFX(FR_DRIVE_ID, CANIVORE_1);
        private static WPI_TalonFX turnFR = new WPI_TalonFX(FR_TURN_ID, CANIVORE_1);
        private static BreakerSwerveAzimuthEncoder encoderFR = new BreakerSwerveLegacyCANcoder(new WPI_CANCoder(FR_ENCODER_ID, CANIVORE_1));

        private static WPI_TalonFX driveBL = new WPI_TalonFX(BL_DRIVE_ID, CANIVORE_1);
        private static WPI_TalonFX turnBL = new WPI_TalonFX(BL_TURN_ID, CANIVORE_1);
        private static BreakerSwerveAzimuthEncoder encoderBL = new BreakerSwerveLegacyCANcoder(new WPI_CANCoder(BL_ENCODER_ID, CANIVORE_1));

        private static WPI_TalonFX driveBR = new WPI_TalonFX(BR_DRIVE_ID, CANIVORE_1);
        private static WPI_TalonFX turnBR = new WPI_TalonFX(BR_TURN_ID, CANIVORE_1);
        private static BreakerSwerveAzimuthEncoder encoderBR = new BreakerSwerveLegacyCANcoder(new WPI_CANCoder(BR_ENCODER_ID, CANIVORE_1));

        private static BreakerSwerveDriveBaseConfig config = new BreakerSwerveDriveBaseConfig(
                MAX_FORWARD_VELOCITY, MAX_SIDEWAYS_VELOCITY, MAX_ANGLE_VELOCITY,
                0.005, 0.05, MODULE_WHEEL_SPEED_DEADBAND, MAX_ATTAINABLE_MODULE_WHEEL_SPEED,
                new PIDController(4.5, 0.0, 0), new PIDController(4.5, 0.0, 0), new PIDController(2.75, 0.0, 0.0))
                .setSlowModeMultipliers(SLOW_MODE_LINEAR_MULTIPLIER, SLOW_MODE_TURN_MULTIPLIER);

        private static BreakerSwerveModuleConfig falconMk4iConfig = new BreakerSwerveModuleConfig(
                DRIVE_MOTOR_GEAR_RATIO_TO_ONE, 1.0, Units.inchesToMeters(WHEEL_DIAMETER), 40.0, 80.0, MODULE_ANGLE_PID_CONFIG, 
                MODULE_VELOCITY_PID_CONFIG, MODULE_VELOCITY_FF);

        private static BreakerSwerveModule frontLeftModule = BreakerSwerveModuleBuilder.getInstance(falconMk4iConfig)
                .withLegacyFalconAngleMotor(turnFL, encoderFL, FL_ENCODER_OFFSET, true)
                .withLegacyFalconDriveMotor(driveFL, true)
                .createSwerveModule(FL_TRANSLATION);

        private static BreakerSwerveModule frontRightModule = BreakerSwerveModuleBuilder.getInstance(falconMk4iConfig)
                .withLegacyFalconAngleMotor(turnFR, encoderFR, FR_ENCODER_OFFSET, true)
                .withLegacyFalconDriveMotor(driveFR, false)
                .createSwerveModule(FR_TRANSLATION);

        private static BreakerSwerveModule backLeftModule = BreakerSwerveModuleBuilder.getInstance(falconMk4iConfig)
                .withLegacyFalconAngleMotor(turnBL, encoderBL, BL_ENCODER_OFFSET, true)
                .withLegacyFalconDriveMotor(driveBL, true)
                .createSwerveModule(BL_TRANSLATION);

        private static BreakerSwerveModule backRightModule = BreakerSwerveModuleBuilder.getInstance(falconMk4iConfig)
                .withLegacyFalconAngleMotor(turnBR, encoderBR, BR_ENCODER_OFFSET, true)
                .withLegacyFalconDriveMotor(driveBR, false)
                .createSwerveModule(BR_TRANSLATION);

        public Drive(BreakerLegacyPigeon2 imu) {
                super(config,  imu, frontLeftModule, frontRightModule, backLeftModule, backRightModule);

                autoDriveController.setTolerances(new Pose2d(0.07, 0.07, Rotation2d.fromDegrees(5.0)));

                frontLeftModule.setDeviceName(" FL_Module ");
                frontRightModule.setDeviceName(" FR_Module ");
                backLeftModule.setDeviceName(" BL_Module ");
                backRightModule.setDeviceName(" BR_Module ");
                BreakerDashboard.getDiagnosticsTab().add("FL Module", frontLeftModule);
                BreakerDashboard.getDiagnosticsTab().add("FR Module", frontRightModule);
                BreakerDashboard.getDiagnosticsTab().add("BL Module", backLeftModule);
                BreakerDashboard.getDiagnosticsTab().add("BR Module", backRightModule);
        }

        @Override
        public void periodic() {
                // TODO Auto-generated method stub
                super.periodic();
                SmartDashboard.putNumber("IMU YAW", getOdometryPoseMeters().getRotation().getDegrees());
                SmartDashboard.putString("ODOMETER", getOdometryPoseMeters().toString());
                // if (DriverStation.getAlliance() == Alliance.Red) {
                //         setFieldRelativeMovementOffsetAngle(Rotation2d.fromDegrees(-180));
                // } else {
                //         setFieldRelativeMovementOffsetAngle(new Rotation2d());
                // }
        }

        public static BreakerWaypointPath mirrorPathToAlliance(BreakerWaypointPath path) {
                if (DriverStation.getAlliance() == Alliance.Red) {
                        return path.mirror(16.54 / 2);
                }
                return path;
        }

        public static BreakerPoseWaypointPath mirrorPathToAlliance(BreakerPoseWaypointPath path) {
                if (DriverStation.getAlliance() == Alliance.Red) {
                        return path.mirror(16.54 / 2);
                }
                return path;
        }
}
