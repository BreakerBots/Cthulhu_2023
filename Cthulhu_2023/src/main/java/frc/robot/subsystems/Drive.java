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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BreakerLib.auto.waypoint.BreakerPoseWaypointPath;
import frc.robot.BreakerLib.auto.waypoint.BreakerSwerveWaypointFollowerConfig;
import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
import frc.robot.BreakerLib.control.BreakerHolonomicDriveController;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerMK4iFalconSwerveModule;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.MiscConstants.*;

/** Add your docs here. */
public class Drive extends BreakerSwerveDrive {

        public static ProfiledPIDController autoAnglePID = new ProfiledPIDController(6.9, 0.0, 0.0,
                        new TrapezoidProfile.Constraints(3.0, 3.0));
        public static PIDController autoDrivePID = new PIDController(6.0, 0.0, 0.0);
        public static BreakerHolonomicDriveController autoDriveController = new BreakerHolonomicDriveController(autoDrivePID, autoAnglePID);
        public BreakerSwerveWaypointFollowerConfig autoConfig = new BreakerSwerveWaypointFollowerConfig(this, autoDriveController);

        private static WPI_TalonFX driveFL = new WPI_TalonFX(FL_DRIVE_ID, CANIVORE_1);
        private static WPI_TalonFX turnFL = new WPI_TalonFX(FL_TURN_ID, CANIVORE_1);
        private static WPI_CANCoder encoderFL = new WPI_CANCoder(FL_ENCODER_ID, CANIVORE_1);

        private static WPI_TalonFX driveFR = new WPI_TalonFX(FR_DRIVE_ID, CANIVORE_1);
        private static WPI_TalonFX turnFR = new WPI_TalonFX(FR_TURN_ID, CANIVORE_1);
        private static WPI_CANCoder encoderFR = new WPI_CANCoder(FR_ENCODER_ID, CANIVORE_1);

        private static WPI_TalonFX driveBL = new WPI_TalonFX(BL_DRIVE_ID, CANIVORE_1);
        private static WPI_TalonFX turnBL = new WPI_TalonFX(BL_TURN_ID, CANIVORE_1);
        private static WPI_CANCoder encoderBL = new WPI_CANCoder(BL_ENCODER_ID, CANIVORE_1);

        private static WPI_TalonFX driveBR = new WPI_TalonFX(BR_DRIVE_ID, CANIVORE_1);
        private static WPI_TalonFX turnBR = new WPI_TalonFX(BR_TURN_ID, CANIVORE_1);
        private static WPI_CANCoder encoderBR = new WPI_CANCoder(BR_ENCODER_ID, CANIVORE_1);

        private static BreakerSwerveDriveConfig config = new BreakerSwerveDriveConfig(
                        MAX_FORWARD_VELOCITY, MAX_SIDEWAYS_VELOCITY, MAX_ANGLE_VELOCITY,
                        MODULE_ANGLE_KP, MODULE_ANGLE_KI, MODULE_ANGLE_KD,
                        MODULE_VELOCITY_KP, MODULE_VELOCITY_KI, MODULE_VELOCITY_KD, 0.0,
                        DRIVE_MOTOR_GEAR_RATIO_TO_ONE, WHEEL_DIAMETER, MODULE_WHEEL_SPEED_DEADBAND,
                        MAX_ATTAINABLE_MODULE_WHEEL_SPEED,
                        new BreakerArbitraryFeedforwardProvider(FF_STATIC_FRICTION_COEFFICIENT,
                                        FF_VELOCITY_COEFFICIENT),
                        FL_TRANSLATION, FR_TRANSLATION, BL_TRANSLATION, BR_TRANSLATION)
                        .setSlowModeMultipliers(SLOW_MODE_LINEAR_MULTIPLIER, SLOW_MODE_TURN_MULTIPLIER);

        @Log
        private static BreakerMK4iFalconSwerveModule frontLeftModule = new BreakerMK4iFalconSwerveModule(driveFL,
                        turnFL,
                        encoderFL, config, FL_ENCODER_OFFSET, true, true);
        private static BreakerMK4iFalconSwerveModule frontRightModule = new BreakerMK4iFalconSwerveModule(driveFR,
                        turnFR,
                        encoderFR, config, FR_ENCODER_OFFSET, false, true);
        private static BreakerMK4iFalconSwerveModule backLeftModule = new BreakerMK4iFalconSwerveModule(driveBL, turnBL,
                        encoderBL, config, BL_ENCODER_OFFSET, true, true);
        private static BreakerMK4iFalconSwerveModule backRightModule = new BreakerMK4iFalconSwerveModule(driveBR,
                        turnBR,
                        encoderBR, config, BR_ENCODER_OFFSET, false, true);

        public Drive(BreakerPigeon2 imu) {
                super(config, imu, frontLeftModule, frontRightModule, backLeftModule, backRightModule);

                autoDriveController.setTolerances(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(1.0)));

                frontLeftModule.setDeviceName(" FL_Module ");
                frontRightModule.setDeviceName(" FR_Module ");
                backLeftModule.setDeviceName(" BL_Module ");
                backRightModule.setDeviceName(" BR_Module ");
                // BreakerDashboard.getDiagnosticsTab().add("FL Module", frontLeftModule);
                // BreakerDashboard.getDiagnosticsTab().add("FR Module", frontRightModule);
                // BreakerDashboard.getDiagnosticsTab().add("BL Module", backLeftModule);
                // BreakerDashboard.getDiagnosticsTab().add("BR Module", backRightModule);
        }

        @Override
        public void periodic() {
                // TODO Auto-generated method stub
                super.periodic();
                SmartDashboard.putNumber("IMU YAW", getOdometryPoseMeters().getRotation().getDegrees());
                SmartDashboard.putString("ODOMETER", getOdometryPoseMeters().toString());
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
