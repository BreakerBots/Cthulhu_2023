// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.SystemDiagnostics;

/** Add your docs here. */
public class Elevator extends SubsystemBase {
    private final TalonFX leftMotor, rightMotor;

    private final MotionMagicDutyCycle motionMagicRequest;
    private final DutyCycleOut dutyCycleRequest;
    private final NeutralOut lockRequest;
    private final CoastOut neutralRequest;

    private final Supplier<ForwardLimitValue> forwardLimit;
    private final Supplier<ReverseLimitValue> reverseLimit;
    private final Supplier<Double> elevatorPosition;
    private final Supplier<Double> elevatorVelocity;

    private final SystemDiagnostics diagnostics;

    private ElevatorState currentState = ElevatorState.CALIBRATEING;
    private boolean hasBeenCalibrated = false;
   
    private double targetHightMeters;
    private double manualControllDutyCycle;

   
    public Elevator() {
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

        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = ElevatorConstants.MAX_ROT;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue =  ElevatorConstants.MIN_ROT;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CUR_LIMIT;
        config.CurrentLimits.SupplyTimeThreshold = ElevatorConstants.SUPPLY_CUR_LIMIT_TIME;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        
        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        targetHightMeters = ElevatorConstants.MIN_HIGHT;

        motionMagicRequest = new MotionMagicDutyCycle(0, false, 0, 0, false);
        dutyCycleRequest = new DutyCycleOut(0, false, false);
        lockRequest = new NeutralOut();
        neutralRequest = new CoastOut();

        forwardLimit = leftMotor.getForwardLimit().asSupplier();
        reverseLimit = leftMotor.getReverseLimit().asSupplier();
        elevatorPosition = leftMotor.getPosition().asSupplier();
        elevatorVelocity = leftMotor.getVelocity().asSupplier();

        diagnostics = new SystemDiagnostics("Elevator");
        diagnostics.addPhoenix6TalonFXs(leftMotor, rightMotor);
    }

    public ElevatorState getCurrentState() {
        return currentState;
    }

    public void setTarget(double hightMeters) {
        targetHightMeters = hightMeters;
        currentState = ElevatorState.AUTOMATIC;
    }

    public void setManual(double dutyCycle) {
        manualControllDutyCycle = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        currentState = ElevatorState.MANUAL;
    }

    public void calibrate() {
        currentState = ElevatorState.CALIBRATEING;
    }

    public double getHight() {
        return elevatorPosition.get();
    }

    public double getVelocity() {
        return elevatorVelocity.get();
    }

    public boolean atTargetHight() {
        return BreakerMath.epsilonEquals(getHight(), targetHightMeters, ElevatorConstants.HIGHT_TOLARENCE) && currentState == ElevatorState.AUTOMATIC;
    }

    public double getTargetHightMeters() {
        return targetHightMeters;
    }

    public boolean getForwardLimitTriggered() {
        return forwardLimit.get() == ForwardLimitValue.ClosedToGround;
    }

    public boolean getReverseLimitTriggered() {
        return reverseLimit.get() == ReverseLimitValue.ClosedToGround;
    }

    public void setNeutral() {
        currentState = ElevatorState.NEUTRAL;
    }

    public void setLocked() {
        currentState = ElevatorState.LOCKED;
    }


    @Override
    public void periodic() {

        if (getForwardLimitTriggered() || getReverseLimitTriggered()) {
            hasBeenCalibrated = true; 
        }

        if (DriverStation.isEnabled()) {
            if (!hasBeenCalibrated) {
                calibrate();
            }
        } else {
            if (hasBeenCalibrated) {
                setLocked();
            } else {
                setNeutral();
            }
        }

        if (DriverStation.isDisabled() || currentState != ElevatorState.AUTOMATIC) {
            targetHightMeters = getHight();
        }

        switch (currentState) { 
            case AUTOMATIC:
                if (targetHightMeters != motionMagicRequest.Position) {
                    leftMotor.setControl(motionMagicRequest.withPosition(Math.min(Math.max(targetHightMeters, ElevatorConstants.MIN_HIGHT), ElevatorConstants.MAX_HIGHT)));
                }
                break;
            case MANUAL:
                leftMotor.setControl(dutyCycleRequest.withOutput(manualControllDutyCycle));
                break;
            case CALIBRATEING:
                leftMotor.setControl(dutyCycleRequest.withOutput(ElevatorConstants.CALIBRAION_DUTY_CYCLE));
                if (getReverseLimitTriggered()) {
                    currentState = ElevatorState.AUTOMATIC;
                    hasBeenCalibrated = true;
                    BreakerLog.logSuperstructureEvent("Elevator zero-point calibration sucessfull");
                }
                break;
            case NEUTRAL:
                leftMotor.setControl(neutralRequest);
                break;
            case LOCKED:
            default:
                leftMotor.setControl(lockRequest);
                break;
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
        public static final int LEFT_MOTOR_ID = 0;
        public static final int RIGHT_MOTOR_ID = 0;
        public static final double SUPPLY_CUR_LIMIT = 60.0;
        public static final double SUPPLY_CUR_LIMIT_TIME = 1.5;

        public static final double CALIBRAION_DUTY_CYCLE = -0.06;
        public static final double HIGHT_TOLARENCE = 0.01;

        public static final double MOTOR_ROT_TO_METERS_SCAILAR = 1.0;
        public static final double MAX_HIGHT = 0.0;
        public static final double MAX_ROT = MAX_HIGHT / MOTOR_ROT_TO_METERS_SCAILAR;
        public static final double MIN_HIGHT = 0.0;
        public static final double MIN_ROT = MIN_HIGHT / MOTOR_ROT_TO_METERS_SCAILAR;
        
    }
}
