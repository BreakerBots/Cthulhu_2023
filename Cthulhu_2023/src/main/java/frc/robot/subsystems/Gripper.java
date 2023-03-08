// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.GripperConstants.COLOR_MATCH_CONFIDENCE_THRESHOLD;
import static frc.robot.Constants.GripperConstants.CONE_COLOR;
import static frc.robot.Constants.GripperConstants.CONE_GRIP_POSITION;
import static frc.robot.Constants.GripperConstants.CUBE_COLOR;
import static frc.robot.Constants.GripperConstants.CUBE_GRIP_POSITION;
import static frc.robot.Constants.GripperConstants.GAME_PIECE_PROX_THRESHOLD;
import static frc.robot.Constants.GripperConstants.GRIP_OPEN_SPD;
import static frc.robot.Constants.GripperConstants.MOTOR_ROT_TO_GRIP_POS_CM;
import static frc.robot.Constants.GripperConstants.*;
import static frc.robot.subsystems.gamepiece.GamePieceType.CONE;
import static frc.robot.subsystems.gamepiece.GamePieceType.CUBE;
import static frc.robot.subsystems.gamepiece.GamePieceType.NONE;

import java.util.Objects;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.devices.sensors.color.BreakerPicoColorSensor;
import frc.robot.BreakerLib.devices.sensors.color.BreakerPicoColorSensor.BreakerPicoColorSensorInstance;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.SystemDiagnostics;
import frc.robot.subsystems.gamepiece.GamePieceType;

/** Add your docs here. */
public class Gripper extends SubsystemBase {

    private BreakerPicoColorSensorInstance colorSensor;
    private CANSparkMax spark;
    private SparkMaxPIDController pid;
    private RelativeEncoder encoder;
    private SystemDiagnostics diagnostics;
    private SparkMaxLimitSwitch limit;
    private double setPosition;
    private boolean isCalibrated = false;
    private ColorMatch colorMatch;
    private BreakerXboxController con;

    public Gripper(BreakerXboxController con) {
        this.con = con;
        spark = new CANSparkMax(Constants.GripperConstants.GRIPPER_MOTOR_ID, MotorType.kBrushless);
        spark.setIdleMode(IdleMode.kBrake);
        spark.setSoftLimit(SoftLimitDirection.kReverse, (float) (MAX_GRIPPER_POSITION_CM * MOTOR_ROT_TO_GRIP_POS_CM));
        spark.enableSoftLimit(SoftLimitDirection.kReverse, true);
        limit = spark.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        limit.enableLimitSwitch(true);
        pid = spark.getPIDController();
        encoder = spark.getEncoder();
        pid.setFeedbackDevice(encoder);
        pid.setP(0);
        pid.setI(0);
        pid.setD(0);
        diagnostics = new SystemDiagnostics("Gripper");
        diagnostics.addSparkMax(spark);
        // setGripperPosition(getGripperPosition());

        colorSensor = new BreakerPicoColorSensor().getSensor0();
        colorMatch = new ColorMatch();
        colorMatch.setConfidenceThreshold(COLOR_MATCH_CONFIDENCE_THRESHOLD);
        colorMatch.addColorMatch(CONE_COLOR);
        colorMatch.addColorMatch(CUBE_COLOR);
        diagnostics.addBreakerDevice(colorSensor);
        spark.burnFlash();
        setOpenGrip();
        encoder.setPosition(0);
    }

    /** @return Gripper position in cm. Larger = more closed. */
    public double getGripperPosition() {
        return encoder.getPosition() * MOTOR_ROT_TO_GRIP_POS_CM;
    }

    /** Moves gripper to set centimeter position. */
    public void setGripperPosition(double position) {
        setPosition = position;
        spark.setSoftLimit(SoftLimitDirection.kReverse, (float) (position * MOTOR_ROT_TO_GRIP_POS_CM));
        spark.enableSoftLimit(SoftLimitDirection.kReverse, true);
        spark.set(-1.0);
    }

    /** Automatically closes grip based on detected game piece type. */
    public void setClosedGrip() {
        setClosedGrip(getControlledGamePieceType());
    }

    /** Closes grip based on given game piece type. */
    public void setClosedGrip(GamePieceType type) {
        switch (type) {
            case CONE:
                setGripperPosition(CONE_GRIP_POSITION);
                break;
            case CUBE:
                setGripperPosition(CUBE_GRIP_POSITION);
                break;
            case NONE:
                setOpenGrip();
                break;
        }
    }

    /**
     * @return Whether the SPARK MAX encoder has been properly calibrated with the
     *         limit switch.
     */
    public boolean isCalibrated() {
        return isCalibrated;
    }

    /** Moves the gripper out until it hits a limit switch. */
    public void setOpenGrip() {
        setGripperSpeed(GRIP_OPEN_SPD);
    }

    /** Moves gripper at given percent speed. */
    public void setGripperSpeed(double percentSpeed) {
        setPosition = 0;
        spark.set(percentSpeed);
    }

    /**
     * @return If the gripper is around its target position or is open and at
     *         position 0.
     */
    public boolean atTargetPosition() {
        return BreakerMath.epsilonEquals(getGripperPosition(), setPosition, 0.2) || (setPosition == 0 && isOpen());
    }

    /** @return If the limit switch is pressed. */
    public boolean isOpen() {
        return limit.isPressed();
    }

    /** @return Detected game piece color from color sensor. */
    public Color getDetectedColor() {
        return colorSensor.getColor();
    }

    /** @return The type of the game piece picked up by the color sensor. */
    public GamePieceType getControlledGamePieceType() {
        ColorMatchResult match = colorMatch.matchColor(colorSensor.getColor());
        if (Objects.nonNull(match)
                && colorSensor.getProximity() <= GAME_PIECE_PROX_THRESHOLD) {
            if (match.color.equals(CONE_COLOR)) {
                return CONE;
            } else {
                return CUBE;
            }
        }
        return NONE;
    }

    @Override
    public void periodic() {
        if (limit.isPressed()) {
            isCalibrated = true;
            spark.enableSoftLimit(SoftLimitDirection.kReverse, true);
            encoder.setPosition(0);
        }

        SmartDashboard.putNumber("SPARK OUT", spark.get());

        //MOTOR CONTROL LOOP!
        if (!atTargetPosition()) {
            spark.set(Math.signum(encoder.getPosition() - setPosition));
        } else {
            spark.set(0);
        }
    }
}
