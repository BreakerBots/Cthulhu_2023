// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveBase.BreakerSwerveDriveBaseConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModuleBuilder.BreakerSwerveModuleConfig;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/** Add your docs here. */
public class OffseasionBotConstants {

    public static final class DriveConstants {
        // Drive motor IDs
        public static final int FL_DRIVE_ID = 10;
        public static final int FR_DRIVE_ID = 12;
        public static final int BL_DRIVE_ID = 14;
        public static final int BR_DRIVE_ID = 16;

        //Azimuth motor IDs
        public static final int FL_TURN_ID = 11;
        public static final int FR_TURN_ID = 13;
        public static final int BL_TURN_ID = 15;
        public static final int BR_TURN_ID = 17;

        //Azimuth Encoder IDs
        public static final int FL_ENCODER_ID = 20;
        public static final int FR_ENCODER_ID = 21;
        public static final int BL_ENCODER_ID = 22;
        public static final int BR_ENCODER_ID = 23;

        //Azimuth encoder angle offets (degrees)
        public static final double FL_ENCODER_OFFSET = 0.0;
        public static final double FR_ENCODER_OFFSET = 0.0;
        public static final double BL_ENCODER_OFFSET = -0.0;
        public static final double BR_ENCODER_OFFSET = -0.0;
    
        //Module wheel centerpoint locations relative to robot origin (center)
        public static final Translation2d FL_TRANSLATION = new Translation2d(0.2635, 0.2635);
        public static final Translation2d FR_TRANSLATION = new Translation2d(0.2635, -0.2635);
        public static final Translation2d BL_TRANSLATION = new Translation2d(-0.2635, 0.2635);
        public static final Translation2d BR_TRANSLATION = new Translation2d(-0.2635, -0.2635);

        //Module Azimuth PIDF constants
        public static final double MODULE_AZIMUTH_KP = 0.85;
        public static final double MODULE_AZIMUTH_KI = 0.0;
        public static final double MODULE_AZIMUTH_KD = 0.0;
        public static final double MODULE_AZIMUTH_KF = 0.0;
        public static final BreakerSwerveMotorPIDConfig MODULE_ANGLE_PID_CONFIG = new BreakerSwerveMotorPIDConfig(MODULE_AZIMUTH_KP, MODULE_AZIMUTH_KI, MODULE_AZIMUTH_KD, MODULE_AZIMUTH_KF);

        //Module Drive Velocity PIDF constants
        public static final double MODULE_VELOCITY_KP = -0.045; // 0.01
        public static final double MODULE_VELOCITY_KI = 0.0;
        public static final double MODULE_VELOCITY_KD = 0.0;
        public static final double MODULE_VELOCITY_KF = 0.0;
        public static final BreakerSwerveMotorPIDConfig MODULE_VELOCITY_PID_CONFIG = new BreakerSwerveMotorPIDConfig(MODULE_VELOCITY_KP, MODULE_VELOCITY_KI, MODULE_VELOCITY_KD, MODULE_VELOCITY_KF);

        //Module Drive Arbitrary FeedForward
        public static final double FF_STATIC_FRICTION_COEFFICIENT = 0.3;
        public static final double FF_VELOCITY_COEFFICIENT = 2.82;
        public static final BreakerArbitraryFeedforwardProvider MODULE_VELOCITY_FF = new BreakerArbitraryFeedforwardProvider(FF_STATIC_FRICTION_COEFFICIENT, FF_VELOCITY_COEFFICIENT);

        //Module physical constants
        public static final double MAX_ATTAINABLE_MODULE_WHEEL_SPEED = 4.2;
        public static final double DRIVE_MOTOR_GEAR_RATIO_TO_ONE = 8.14;
        public static final double AZIMUTH_MOTOR_GEAR_RATIO_TO_ONE = 1.0;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double MODULE_WHEEL_SPEED_DEADBAND = 0.001;
        public static final double AZIMUTH_MOTOR_SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double DRIVE_MOTOR_SUPPLY_CURRENT_LIMIT = 80.0;
        public static final BreakerSwerveModuleConfig MODULE_CONFIG = new BreakerSwerveModuleConfig(
            DRIVE_MOTOR_GEAR_RATIO_TO_ONE, AZIMUTH_MOTOR_GEAR_RATIO_TO_ONE, 
            WHEEL_DIAMETER, 
            AZIMUTH_MOTOR_SUPPLY_CURRENT_LIMIT, DRIVE_MOTOR_SUPPLY_CURRENT_LIMIT, 
            MODULE_ANGLE_PID_CONFIG, MODULE_VELOCITY_PID_CONFIG, 
            MODULE_VELOCITY_FF
            );

        //X-axis positional PID
        public static final double X_PID_KP = 4.5;
        public static final double X_PID_KI = 0.0;
        public static final double X_PID_KD = 0.0;
        public static final PIDController X_PID = new PIDController(X_PID_KP, X_PID_KI, X_PID_KD);

        //Y-axis positional PID
        public static final double Y_PID_KP = 4.5;
        public static final double Y_PID_KI = 0.0;
        public static final double Y_PID_KD = 0.0;
        public static final PIDController Y_PID = new PIDController(Y_PID_KP, Y_PID_KI, Y_PID_KD);

        //Theta-axis positional PID
        public static final double THETA_PID_KP = 4.5;
        public static final double THETA_PID_KI = 0.0;
        public static final double THETA_PID_KD = 0.0;
        public static final PIDController THETA_PID = new PIDController(THETA_PID_KP, THETA_PID_KI, THETA_PID_KD);

        //Slow mode constants
        public static final double SLOW_MODE_LINEAR_MULTIPLIER = 0.5;
        public static final double SLOW_MODE_TURN_MULTIPLIER = 0.5;

        //Physical Robot Constants
        public static final double MAX_ANGULAR_VEL = ((FL_TRANSLATION.getNorm() * 2.0 * Math.PI) / MAX_ATTAINABLE_MODULE_WHEEL_SPEED) * (2.0 * Math.PI); 
        public static final double MAX_LINEAR_VEL = 4.2;
        public static final double HEADING_COMPENSATION_ANGULAR_VEL_DEADBAND = 0.005;
        public static final double HEADING_COMPENSATION_MIN_ACTIVE_LINEAR_VEL = 0.01;
        public static final BreakerSwerveDriveBaseConfig DRIVE_BASE_CONFIG = new BreakerSwerveDriveBaseConfig(
        MAX_LINEAR_VEL, MAX_LINEAR_VEL, MAX_ANGULAR_VEL, 
        HEADING_COMPENSATION_ANGULAR_VEL_DEADBAND, HEADING_COMPENSATION_MIN_ACTIVE_LINEAR_VEL, 
        MODULE_WHEEL_SPEED_DEADBAND, MAX_ATTAINABLE_MODULE_WHEEL_SPEED, 
        X_PID, Y_PID, THETA_PID)
        .setSlowModeMultipliers(SLOW_MODE_LINEAR_MULTIPLIER, SLOW_MODE_TURN_MULTIPLIER);
    }

    public static final class ElevatorConstants {
        //General Motor Configs
        public static final int LEFT_MOTOR_ID = 30;
        public static final int RIGHT_MOTOR_ID = 31;
        public static final double SUPPLY_CUR_LIMIT = 60.0;
        public static final double SUPPLY_CUR_LIMIT_TIME = 1.5;

        //Motion Magic Configs
        public static final double MOTION_MAGIC_CRUISE_VEL = 0;
        public static final double MOTION_MAGIC_ACCEL = 0;
        public static final double MOTION_MAGIC_JERK = 0;

        //PIDF Configs
        public static final double PIDF_KP = 0;
        public static final double PIDF_KI = 0;
        public static final double PIDF_KD = 0;
        public static final double PIDF_KS = 0;
        public static final double PIDF_KV = 0;

        //Gearing
        public static final double MOTOR_TO_DRUM_GEARING = 100.0; //to one
        public static final double DRUM_RADIUS_METERS = 0.02;
        public static final double DRUM_CIRCUMFERENCE_METERS = 2*Math.PI*DRUM_RADIUS_METERS;
        public static final double MOTOR_ROT_TO_METERS_SCALAR = DRUM_CIRCUMFERENCE_METERS / MOTOR_TO_DRUM_GEARING;

        //Physical Limits
        public static final double MAX_HEIGHT = 1.0;
        public static final double MAX_ROT = MAX_HEIGHT / MOTOR_ROT_TO_METERS_SCALAR;
        public static final double MIN_HEIGHT = 0.0;
        public static final double MIN_ROT = MIN_HEIGHT / MOTOR_ROT_TO_METERS_SCALAR;

        //Sim Configs
        public static final double CARRIAGE_MASS_KG = 15.0;
        public static final boolean SIM_GRAVITY = false;
        public static final ChassisReference MOTOR_CHASSIS_REF = ChassisReference.CounterClockwise_Positive;
        
        //Misc
        public static final double CALIBRATION_DUTY_CYCLE = -0.06;
        public static final double HIGHT_TOLARENCE = 0.01;

        //Command configs
        public static final double MOVE_TO_HEIGHT_COMMAND_TIMEOUT = 5.0;
    
    }

    public static final class IntakeConstants {
        public static final int ACTUATOR_ID = 40;
        public static final int ROLLER_ID = 41;
        public static final int BEAM_BRAKE_DIO_PORT = 0;
        public static final boolean BEAM_BRAKE_BROKEN_ON_TRUE = true;
    
        public static final double ACTUATOR_CURRENT_LIMIT = 0;
        public static final double ACTUATOR_CURRENT_LIMIT_TIME = 0;
        public static final double ROLLER_CURRENT_LIMIT = 0;
        public static final double ROLLER_CURRENT_LIMIT_TIME = 0;
    
        public static final double ACTUATOR_EXTEND_DUTY_CYCLE = 0;
        public static final double ACTUATOR_RETRACT_DUTY_CYCLE = 0;
    
        public static final double INTAKE_DUTY_CYCLE = 0;
        public static final double INTAKE_GRIP_DUTY_CYCLE = 0;
        public static final double EXTAKE_DUTY_CYCLE = 0;
    
        public static final double EJECT_COMMAND_WAIT_FOR_EXTEND_TIMEOUT = 0;
        public static final double EJECT_COMMAND_CUTOFF_TRALING_DELAY = 0;
        public static final double EJECT_COMMAND_CUTOFF_TIMEOUT = 0;
      }

    public static class VisionConstants {
        public static final String FRONT_CAMERA_NAME = "frontCam";
        public static final String LEFT_CAMERA_NAME = "leftCam";
        public static final String RIGHT_CAMERA_NAME = "rightCam";
        public static final String BACK_CAMERA_NAME = "backCam";
    
        public static final Transform3d FRONT_CAMERA_POSE = new Transform3d();
        public static final Transform3d LEFT_CAMERA_POSE = new Transform3d();
        public static final Transform3d RIGHT_CAMERA_POSE = new Transform3d();
        public static final Transform3d BACK_CAMERA_POSE = new Transform3d();
    }

    public static final class MiscConstants {
        public static final int IMU_ID = 5;
        public static final String CANIVORE_1 = "CANivore_1";
    }
}
