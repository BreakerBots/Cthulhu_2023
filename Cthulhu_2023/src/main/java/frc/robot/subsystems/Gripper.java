// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Objects;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.devices.sensors.color.BreakerPicoColorSensor;
import frc.robot.BreakerLib.devices.sensors.color.BreakerPicoColorSensor.BreakerPicoColorSensorInstance;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.SystemDiagnostics;

/** Add your docs here. */
public class Gripper extends SubsystemBase {

    enum GrippedGamePieceType {
        CONE,
        CUBE,
        NONE
    }

    private BreakerPicoColorSensorInstance colorSensor;
    private CANSparkMax spark;
    private SparkMaxPIDController pid;
    private RelativeEncoder encoder;
    private SystemDiagnostics diagnostics;
    private SparkMaxLimitSwitch limit;
    private double setPosition;
    private boolean isCalibrated = false;
    private ColorMatch colorMatch;

    public Gripper() {
        spark = new CANSparkMax(0, MotorType.kBrushless);
        spark.setIdleMode(IdleMode.kBrake);
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
        setGripperPosition(getGripperPosition());

        colorSensor = new BreakerPicoColorSensor().getSensor0();
        colorMatch = new ColorMatch();
        colorMatch.setConfidenceThreshold(Constants.GripperConstants.COLOR_MATCH_CONFIDENCE_THRESHOLD);
        colorMatch.addColorMatch(Constants.GripperConstants.CONE_COLOR);
        colorMatch.addColorMatch(Constants.GripperConstants.CUBE_COLOR);
        diagnostics.addBreakerDevice(colorSensor);
    }

    /** @return Gripper position in cm. Larger = more closed. */
    public double getGripperPosition() {
        return encoder.getPosition() * Constants.GripperConstants.MOTOR_ROT_TO_GRIP_POS_CM;
    }

    /** Moves gripper to set centimeter position. */
    public void setGripperPosition(double position) {
        setPosition = position;
        pid.setReference(position / Constants.GripperConstants.MOTOR_ROT_TO_GRIP_POS_CM, ControlType.kPosition);
    }

    /** Automatically closes grip based on detected game piece type. */
    public void setClosedGrip() {
        setClosedGrip(getControlledGamePieceType());
    }

    /** Closes grip based on given game piece type. */
    public void setClosedGrip(GrippedGamePieceType type) {
        switch (type) {
            case CONE:
                setGripperPosition(Constants.GripperConstants.CONE_GRIP_POSITION);
                break;
            case CUBE:
                setGripperPosition(Constants.GripperConstants.CUBE_GRIP_POSITION);
                break;
            case NONE:
                setOpenGrip();
                break;
        }
    }

    /** @return Whether the SPARK MAX encoder has been properly calibrated with the limit switch. */
    public boolean isCalibrated() {
        return isCalibrated;
    }

    /** Moves the gripper out until it hits a limit switch. */
    public void setOpenGrip() {
        setGripperSpeed(Constants.GripperConstants.GRIP_OPEN_SPD);
    }

    /** Moves gripper at given percent speed. */
    public void setGripperSpeed(double percentSpeed) {
        setPosition = 0;
        spark.set(percentSpeed);
    }

    /** @return If the gripper is around its target position or is open and at position 0. */
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
    public GrippedGamePieceType getControlledGamePieceType() {
        ColorMatchResult match = colorMatch.matchColor(colorSensor.getColor());
        if (Objects.nonNull(match)
                && colorSensor.getProximity() <= Constants.GripperConstants.GAME_PIECE_PROX_THRESHOLD) {
            if (match.color.equals(Constants.GripperConstants.CONE_COLOR)) {
                return GrippedGamePieceType.CONE;
            } else {
                return GrippedGamePieceType.CUBE;
            }
        }
        return GrippedGamePieceType.NONE;
    }

    @Override
    public void periodic() {
        if (limit.isPressed()) {
            isCalibrated = true;
            encoder.setPosition(0.0);
        }
    }
}
