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

    enum GrippedPieceType {
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

    public double getGripperPosition() {
        return encoder.getPosition() * Constants.GripperConstants.MOTOR_ROT_TO_GRIP_POS_CM;
    }

    public void setGripperPosition(double position) {
        setPosition = position;
        pid.setReference(position / Constants.GripperConstants.MOTOR_ROT_TO_GRIP_POS_CM, ControlType.kPosition);
    }

    public void setClosedGrip() {
        setClosedGrip(getControlledGamePieceType());
    }

    public void setClosedGrip(GrippedPieceType type) {
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

    public boolean isCalibrated() {
        return isCalibrated;
    }

    public void setOpenGrip() {
        setGripperSpeed(Constants.GripperConstants.GRIP_OPEN_SPD);
    }

    public void setGripperSpeed(double precentSpeed) {
        setPosition = 0;
        spark.set(precentSpeed);
    }

    public boolean atTargetPosition() {
        return BreakerMath.epsilonEquals(getGripperPosition(), setPosition, 0.2) || (setPosition == 0 && isOpen());
    }

    public boolean isOpen() {
        return limit.isPressed();
    }

    public Color getDetectedColor() {
        return colorSensor.getColor();
    }

    public GrippedPieceType getControlledGamePieceType() {
        ColorMatchResult match = colorMatch.matchColor(colorSensor.getColor());
        if (Objects.nonNull(match)
                && colorSensor.getProximity() <= Constants.GripperConstants.GAME_PIECE_PROX_THRESHOLD) {
            if (match.color.equals(Constants.GripperConstants.CONE_COLOR)) {
                return GrippedPieceType.CONE;
            } else {
                return GrippedPieceType.CUBE;
            }
        }
        return GrippedPieceType.NONE;
    }

    @Override
    public void periodic() {
        if (limit.isPressed()) {
            isCalibrated = true;
            encoder.setPosition(0.0);
        }
    }
}
