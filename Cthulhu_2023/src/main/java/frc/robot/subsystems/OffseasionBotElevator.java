// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class OffseasionBotElevator extends SubsystemBase {
    private final TalonFX leftMotor, rightMotor;

    private final MotionMagicDutyCycle motionMagicRequest;
    private final DutyCycleOut dutyCycleRequest;
    private final NeutralOut lockRequest;
    private final CoastOut neutralRequest;

    private final Supplier<ForwardLimitValue> forwardLimit;
    private final Supplier<ReverseLimitValue> reverseLimit;
    private final Supplier<Double> elevatorPosition;

    private ElevatorState currentState = ElevatorState.CALIBRATEING;
    private boolean hasBeenCalibrated = false;
    private boolean dynamicCalibrationEnabled = true;

    private double targetHightMeters;
    private double manualControllDutyCycle;

   
    public OffseasionBotElevator() {
        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, "placeholder");
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, "placeholder");
        leftMotor.setControl(new Follower(ElevatorConstants.RIGHT_MOTOR_ID, false));
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        config.MotionMagic.MotionMagicAcceleration= 0.0;
        config.MotionMagic.MotionMagicJerk = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kP = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.MOTOR_ROT_TO_METERS_SCAILAR;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CUR_LIMIT;
        config.CurrentLimits.SupplyTimeThreshold = ElevatorConstants.SUPPLY_CUR_LIMIT_TIME;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        
        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        targetHightMeters = 0.0;

        motionMagicRequest = new MotionMagicDutyCycle(0, false, 0, 0, false);
        dutyCycleRequest = new DutyCycleOut(0, false, false);
        lockRequest = new NeutralOut();
        neutralRequest = new CoastOut();

        forwardLimit = leftMotor.getForwardLimit().asSupplier();
        reverseLimit = leftMotor.getReverseLimit().asSupplier();
        elevatorPosition = leftMotor.getPosition().asSupplier();

    }

    public ElevatorState getCurrentState() {
        return currentState;
    }


    public void setTarget(double hightMeters) {
        targetHightMeters = hightMeters;
        currentState = ElevatorState.AUTOMATIC;
    }

    public void setManual(double dutyCycle) {
        manualControllDutyCycle = dutyCycle;
        currentState = ElevatorState.MANUAL;
    }

    public void calibrate() {
        currentState = ElevatorState.CALIBRATEING;
    }

    public double getHight() {
        return elevatorPosition.get();
    }

    public boolean atTargetHight() {
        return BreakerMath.epsilonEquals(getHight(), targetHightMeters, ElevatorConstants.HIGHT_TOLARENCE) && currentState == ElevatorState.AUTOMATIC;
    }

    public double getTargetHightMeters() {
        return targetHightMeters;
    }


    @Override
    public void periodic() {
        if (!hasBeenCalibrated) {
            calibrate();
        }
        if (DriverStation.isDisabled() || currentState != ElevatorState.AUTOMATIC) {
            targetHightMeters = getHight();
        }
        switch (currentState) { 
            case AUTOMATIC:
                leftMotor.setControl(motionMagicRequest.withPosition(Math.min(Math.max(targetHightMeters, 0.0), ElevatorConstants.MAX_HIGHT)));
                break;
            case MANUAL:
                leftMotor.setControl(dutyCycleRequest.withOutput(manualControllDutyCycle));
                break;
            case CALIBRATEING:
                leftMotor.setControl(dutyCycleRequest.withOutput(ElevatorConstants.CALIBRAION_DUTY_CYCLE));
                if (reverseLimit.get() == ReverseLimitValue.ClosedToGround) {
                    currentState = ElevatorState.AUTOMATIC;
                    hasBeenCalibrated = true;
                }
                break;
            case LOCKED:
                leftMotor.setControl(lockRequest);
                break;
            case NEUTRAL:
                leftMotor.setControl(neutralRequest);
                break;
            default:
                break;
        }
        if (dynamicCalibrationEnabled) {
            if (forwardLimit.get() == ForwardLimitValue.ClosedToGround) {
                leftMotor.setRotorPosition(ElevatorConstants.MAX_HIGHT/ElevatorConstants.MOTOR_ROT_TO_METERS_SCAILAR);
                BreakerLog.logSuperstructureEvent("Elevator dynamicly recalibrated (fwd lim)");
            }
            if (reverseLimit.get() == ReverseLimitValue.ClosedToGround) {
                leftMotor.setRotorPosition(0.0);
                BreakerLog.logSuperstructureEvent("Elevator dynamicly recalibrated (rev lim)");
            }
        }
    }

    private static enum ElevatorState {
        CALIBRATEING,
        AUTOMATIC,
        MANUAL,
        LOCKED,
        NEUTRAL
    }

    public static final class ElevatorConstants {
        static final int LEFT_MOTOR_ID = 0;
        static final int RIGHT_MOTOR_ID = 0;
        static final double SUPPLY_CUR_LIMIT = 60.0;
        static final double SUPPLY_CUR_LIMIT_TIME = 1.5;

        static final double CALIBRAION_DUTY_CYCLE = -0.06;
        static final double HIGHT_TOLARENCE = 0.01;

        static final double MAX_HIGHT = 0.0;
        static final double MOTOR_ROT_TO_METERS_SCAILAR = 0.0;
    }
}
