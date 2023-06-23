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
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
    private final Supplier<Double> elevatorPosition, elevatorVelocity;

    private final SystemDiagnostics diagnostics;

    private ElevatorState currentState = ElevatorState.CALIBRATING;
    private boolean hasBeenCalibrated = false;
   
    private double targetHeightMeters;
    private double manualControlDutyCycle;

    private ElevatorSimManager simManager;

   
    public Elevator() {
        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, "placeholder");
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, "placeholder");
        leftMotor.setControl(new Follower(ElevatorConstants.RIGHT_MOTOR_ID, false));
        
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_MAGIC_CRUISE_VEL;
        config.MotionMagic.MotionMagicAcceleration= ElevatorConstants.MOTION_MAGIC_ACCEL;
        config.MotionMagic.MotionMagicJerk = ElevatorConstants.MOTION_MAGIC_JERK;

        config.Slot0.kP = ElevatorConstants.PIDF_KP;
        config.Slot0.kI = ElevatorConstants.PIDF_KI;
        config.Slot0.kD = ElevatorConstants.PIDF_KD;
        config.Slot0.kS = ElevatorConstants.PIDF_KS;
        config.Slot0.kV = ElevatorConstants.PIDF_KV;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.MOTOR_ROT_TO_METERS_SCALAR;

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

        targetHeightMeters = ElevatorConstants.MIN_HEIGHT;

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

        simManager = this.new ElevatorSimManager();
        
    }

    public ElevatorState getCurrentState() {
        return currentState;
    }

    public void setTarget(double heightMeters) {
        targetHeightMeters = heightMeters;
        currentState = ElevatorState.AUTOMATIC;
    }

    public void setManual(double dutyCycle) {
        manualControlDutyCycle = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        currentState = ElevatorState.MANUAL;
        // System.out.println(manualControlDutyCycle);
    }

    public void calibrate() {
        currentState = ElevatorState.CALIBRATING;
    }

    public double getHeight() {
        return elevatorPosition.get();
    }

    public double getVelocity() {
        return elevatorVelocity.get();
    }

    public boolean atTargetHeight() {
        return BreakerMath.epsilonEquals(getHeight(), targetHeightMeters, ElevatorConstants.HIGHT_TOLARENCE) && currentState == ElevatorState.AUTOMATIC;
    }

    public double getTargetHeightMeters() {
        return targetHeightMeters;
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
    public void simulationPeriodic() {
        simManager.updateSim();
    }

    @Override
    public void periodic() {

        // Elevator calibrates by hitting limit switch and resetting
        if (RobotBase.isReal()) {
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
        }
        

        if (DriverStation.isDisabled() || currentState != ElevatorState.AUTOMATIC) {
            targetHeightMeters = getHeight();
        }

        switch (currentState) { 
            case AUTOMATIC:
                if (targetHeightMeters != motionMagicRequest.Position) {
                    leftMotor.setControl(motionMagicRequest.withPosition(Math.min(Math.max(targetHeightMeters, ElevatorConstants.MIN_HEIGHT), ElevatorConstants.MAX_HEIGHT)));
                }
                break;
            case MANUAL:
                leftMotor.setControl(dutyCycleRequest.withOutput(manualControlDutyCycle));
                break;
            case CALIBRATING:
                if (RobotBase.isReal()) {
                    leftMotor.setControl(dutyCycleRequest.withOutput(ElevatorConstants.CALIBRATION_DUTY_CYCLE));
                    if (getReverseLimitTriggered()) {
                        currentState = ElevatorState.AUTOMATIC;
                        hasBeenCalibrated = true;
                        BreakerLog.logSuperstructureEvent("Elevator zero-point calibration sucessfull");
                    }
                } else {
                    currentState = ElevatorState.AUTOMATIC;
                        hasBeenCalibrated = true;
                        BreakerLog.logSuperstructureEvent("Elevator calibation not supported in sim, action fallthrough");
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
        CALIBRATING,
        AUTOMATIC,
        MANUAL,
        LOCKED,
        NEUTRAL
    }

    public static final class ElevatorConstants {
        //General Motor Configs
        public static final int LEFT_MOTOR_ID = 0;
        public static final int RIGHT_MOTOR_ID = 0;
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
    
    }

    private class ElevatorSimManager {
        private ElevatorSim sim;
        private TalonFXSimState simState;
        private final Timer timer = new Timer();
        private boolean hasBeenInit = false;
        public ElevatorSimManager() {
            sim = new ElevatorSim(
            DCMotor.getFalcon500(2), 
            ElevatorConstants.MOTOR_TO_DRUM_GEARING, 
            ElevatorConstants.CARRIAGE_MASS_KG,
            ElevatorConstants.DRUM_RADIUS_METERS, 
            ElevatorConstants.MIN_HEIGHT,
            ElevatorConstants.MAX_HEIGHT, 
            ElevatorConstants.SIM_GRAVITY);
            simState = leftMotor.getSimState();
        }

        private void initSim() {
            simState.setRawRotorPosition(ElevatorConstants.MAX_ROT);
            simState.Orientation = ElevatorConstants.MOTOR_CHASSIS_REF;
            timer.restart();
        }

        public void updateSim() {
            if (!hasBeenInit) {
                initSim();
            }

            double elapsedTime = timer.get();
            timer.reset();

            simState.setSupplyVoltage(RobotController.getBatteryVoltage());
            sim.setInput(12);
            sim.update(elapsedTime);
            simState.setRawRotorPosition(sim.getPositionMeters() / ElevatorConstants.MOTOR_ROT_TO_METERS_SCALAR);
            simState.setRotorVelocity(sim.getVelocityMetersPerSecond() / ElevatorConstants.MOTOR_ROT_TO_METERS_SCALAR);
            System.out.println(sim.getPositionMeters());
        }
    }
}
