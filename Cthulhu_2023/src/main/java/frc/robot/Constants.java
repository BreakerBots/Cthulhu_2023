// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.awt.geom.Rectangle2D;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.position.geometry.BreakerRobotGeometry2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Static class containing necessary functions for driving.
     */
    public static final class DriveConstants {

        public static final double MAX_FORWARD_VELOCITY = 4.1148;
        public static final double MAX_SIDEWAYS_VELOCITY = 4.1148;
        public static final double MAX_ANGLE_VELOCITY = 16.114;
        public static final double MODULE_ANGLE_KP = 0.85;
        public static final double MODULE_ANGLE_KI = 0.0;
        public static final double MODULE_ANGLE_KD = 0.0;
        public static final double MODULE_VELOCITY_KP = -0.045; // 0.01
        public static final double MODULE_VELOCITY_KI = 0.0;
        public static final double MODULE_VELOCITY_KD = 0.0;
        public static final double DRIVE_MOTOR_GEAR_RATIO_TO_ONE = 8.14;
        public static final double WHEEL_DIAMETER = 4.0;
        public static final double MODULE_WHEEL_SPEED_DEADBAND = 0.001;
        public static final double MAX_ATTAINABLE_MODULE_WHEEL_SPEED = 4.2;

        public static final double FF_STATIC_FRICTION_COEFFICIENT = 0.3;
        public static final double FF_VELOCITY_COEFFICIENT = 2.82;

        public static final double SLOW_MODE_LINEAR_MULTIPLIER = 0.5;
        public static final double SLOW_MODE_TURN_MULTIPLIER = 0.5;

        public static final int FL_DRIVE_ID = 10;
        public static final int FR_DRIVE_ID = 12;
        public static final int BL_DRIVE_ID = 14;
        public static final int BR_DRIVE_ID = 16;

        public static final int FL_TURN_ID = 11;
        public static final int FR_TURN_ID = 13;
        public static final int BL_TURN_ID = 15;
        public static final int BR_TURN_ID = 17;

        public static final int FL_ENCODER_ID = 20;
        public static final int FR_ENCODER_ID = 21;
        public static final int BL_ENCODER_ID = 22;
        public static final int BR_ENCODER_ID = 23;

        // FL is pos/pos, FR is pos/neg, BL is neg/pos, BR is neg/neg
        public static final Translation2d FL_TRANSLATION = new Translation2d(0.2635, 0.2635);
        public static final Translation2d FR_TRANSLATION = new Translation2d(0.2635, -0.2635);
        public static final Translation2d BL_TRANSLATION = new Translation2d(-0.2635, 0.2635);
        public static final Translation2d BR_TRANSLATION = new Translation2d(-0.2635, -0.2635);

        // Gears all face outward
        public static final double FL_ENCODER_OFFSET = -150.732;
        public static final double FR_ENCODER_OFFSET = 38.5;
        public static final double BL_ENCODER_OFFSET = -85.693;
        public static final double BR_ENCODER_OFFSET = 122.113;

        // public static final double FL_ENCODER_OFFSET = -112.500;
        // public static final double FR_ENCODER_OFFSET = 141.152;
        // public static final double BL_ENCODER_OFFSET = 117.246;
        // public static final double BR_ENCODER_OFFSET = -155.918;

    }

    public static final class VisionConstants {
        public static final class AprilTagConstants {
            // public static final BreakerPhotonCamera[] APRILTAG_CAMERAS = new
            // BreakerPhotonCamera[] {
            // new BreakerPhotonCamera("April_Test_1", new Transform3d(new Translation3d(),
            // new Rotation3d()))
            // new BreakerPhotonCamera("leftApriltagCam", null),
            // new BreakerPhotonCamera("rightApriltagCam", null),
            // new BreakerPhotonCamera("backApriltagCam", null)
            // };

            // public static BreakerFiducialPhotonTarget[] APRILTAGS = new
            // BreakerFiducialPhotonTarget[] {
            // new BreakerFiducialPhotonTarget(1,
            // new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19),
            // Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.toRadians(180))),
            // APRILTAG_CAMERAS),
            // new BreakerFiducialPhotonTarget(2,
            // new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19),
            // Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.toRadians(180))),
            // APRILTAG_CAMERAS),
            // new BreakerFiducialPhotonTarget(3,
            // new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19),
            // Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.toRadians(180))),
            // APRILTAG_CAMERAS),
            // new BreakerFiducialPhotonTarget(4,
            // new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74),
            // Units.inchesToMeters(27.38), new Rotation3d(0, 0, Math.toRadians(180))),
            // APRILTAG_CAMERAS),
            // new BreakerFiducialPhotonTarget(5,
            // new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74),
            // Units.inchesToMeters(27.38), new Rotation3d(0, 0, 0)),
            // APRILTAG_CAMERAS),
            // new BreakerFiducialPhotonTarget(6,
            // new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19),
            // Units.inchesToMeters(18.22), new Rotation3d(0, 0, 0)),
            // APRILTAG_CAMERAS),
            // new BreakerFiducialPhotonTarget(7,
            // new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19),
            // Units.inchesToMeters(18.22), new Rotation3d(0, 0, 0)),
            // APRILTAG_CAMERAS),
            // new BreakerFiducialPhotonTarget(8,
            // new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19),
            // Units.inchesToMeters(18.22), new Rotation3d(0, 0, 0)),
            // APRILTAG_CAMERAS)
            // };
        }
    }

    public static final class RollerIntakeConstants {
        public static final int INTAKE_ID = 32;
    }

    public static final class GripperConstants {
        public static final int GRIPPER_MOTOR_ID = 32;

        public static final double MOTOR_ROT_TO_GRIP_POS_CM = Units.inchesToMeters(-15 * 12) * 100;
        public static final double CONE_GRIP_POSITION = -6.0 * MOTOR_ROT_TO_GRIP_POS_CM;
        public static final double CUBE_GRIP_POSITION = -6.0;
        public static final double ClOSED_GRIP_POSITION = 0.0;
        public static final double MAX_GRIPPER_POSITION_CM = 9;
        public static final double GRIP_OPEN_SPD = 1.0;
        public static final Color CONE_COLOR = new Color(0, 0, 0);
        public static final Color CUBE_COLOR = new Color(0, 0, 0);
        public static final double COLOR_MATCH_CONFIDENCE_THRESHOLD = 95.0;
        public static final double GAME_PIECE_PROX_THRESHOLD = 0.0;
        public static final double AUTO_CLOSE_DELAY_SEC = 0.5;
    }

    public static final class ArmConstants {
        public static final int PROXIMAL_MOTOR_ID = 30;
        public static final int PROXIMAL_ENCODER_ID = 24;
        public static final double PROXIMAL_ENCODER_OFFSET = 107.6;
        public static final double PROX_ARM_LENGTH_METERS = 1;

        public static final int DISTAL_MOTOR_ID = 31;
        public static final int DISTAL_ENCODER_ID = 25;
        public static final double DISTAL_ENCODER_OFFSET = -192.932;
        public static final double DIST_ARM_LENGTH_METERS = 1;
        public static final double DIST_BIAS_ANGLE = -10;

        public static final double PROX_KP = 0.012;
        public static final double PROX_KI = 0;
        public static final double PROX_KD = 0.005;

        public static final double PROX_KS = 0;
        public static final double PROX_KG = 0;
        public static final double PROX_KV = 0;
        public static final double PROX_KA = 0;

        public static final double DIST_KP = 0.012;
        public static final double DIST_KI = 0.0;
        public static final double DIST_KD = 0.0025;

        public static final double DIST_KS = 0;
        public static final double DIST_KG = 0;
        public static final double DIST_KV = 0;
        public static final double DIST_KA = 0;

    }

    public static final class FieldConstants {
        public static final java.awt.geom.Rectangle2D BLUE_CHARGING_STATION = new Rectangle2D.Double(0, 0, 0, 0);
        public static final java.awt.geom.Rectangle2D BLUE_CHARGING_STATION_WORKING_BOUNDS = new Rectangle2D.Double(0,
                0, 0, 0);
        public static final java.awt.geom.Rectangle2D RED_CHARGING_STATION = new Rectangle2D.Double(0, 0, 0, 0);
        public static final java.awt.geom.Rectangle2D RED_CHARGING_STATION_WORKING_BOUNDS = new Rectangle2D.Double(0,
                0, 0, 0);
    }

    public static final class RobotGeometryConstants {
        // TODO: Define this!
        public static final BreakerRobotGeometry2d ROBOT_GEOMETRY = new BreakerRobotGeometry2d(null);
    }

    public static final class MiscConstants {
        public static final int IMU_ID = 5;
        public static final String CANIVORE_1 = "CANivore_1";
        public static final String CANIVORE_2 = "CANivore_2";
    }
}
