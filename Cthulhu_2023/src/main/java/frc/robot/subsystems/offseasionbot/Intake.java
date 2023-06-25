// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;
import frc.robot.BreakerLib.util.test.selftest.SystemDiagnostics;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX actuatorMotor;
  private final TalonFX rollerMotor;
  private final BreakerBeamBreak beamBreak;

  private final DutyCycleOut dutyCycleRequest;
  private final NeutralOut neutralRequest;

  private RollerState rollerState;
  private ActuatorMotorState actuatorMotorState;

  private final SystemDiagnostics diagnostics;
  public Intake() {
    actuatorMotor = new TalonFX(IntakeConstants.ACTUATOR_ID, "placeholder");
    rollerMotor = new TalonFX(IntakeConstants.ROLLER_ID, "placeholder");
    beamBreak = new BreakerBeamBreak(IntakeConstants.BEAM_BRAKE_DIO_PORT, IntakeConstants.BEAM_BRAKE_BROKEN_ON_TRUE);
    
    TalonFXConfiguration actuatorConfig = new TalonFXConfiguration();
    actuatorConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    actuatorConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    actuatorConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    actuatorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    actuatorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    actuatorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    actuatorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ACTUATOR_CURRENT_LIMIT;
    actuatorConfig.CurrentLimits.SupplyTimeThreshold = IntakeConstants.ACTUATOR_CURRENT_LIMIT_TIME;
    actuatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    actuatorMotor.getConfigurator().apply(actuatorConfig);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_CURRENT_LIMIT;
    rollerConfig.CurrentLimits.SupplyTimeThreshold = IntakeConstants.ROLLER_CURRENT_LIMIT_TIME;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    rollerMotor.getConfigurator().apply(rollerConfig);

    rollerState = RollerState.NEUTRAL;
    actuatorMotorState = getActuatorState() == ActuatorState.EXTENDED ? ActuatorMotorState.EXTENDING : ActuatorMotorState.RETRACTING;

    dutyCycleRequest = new DutyCycleOut(0, false, false);
    neutralRequest = new NeutralOut();

    diagnostics = new SystemDiagnostics("Intake");
    diagnostics.addPhoenix6TalonFXs(actuatorMotor, rollerMotor);
  }

  public boolean hasGamePiece() {
    return beamBreak.isBroken();
  }

  public ActuatorMotorState getActuatorMotorState() {
    return actuatorMotorState;
  }

  public boolean isInDesiredActuatorState() {
    return actuatorMotorState == ActuatorMotorState.EXTENDING ? getActuatorState() == ActuatorState.EXTENDED : getActuatorState() == ActuatorState.RETRACTED;
  }

  public ActuatorState getActuatorState() {
    boolean fwdLimit = actuatorMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    boolean revLimit = actuatorMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    if (fwdLimit && !revLimit) {
      return ActuatorState.EXTENDED;
    } else if (!fwdLimit && revLimit) {
      return ActuatorState.RETRACTED;
    } else if (!fwdLimit && !revLimit) {
      return ActuatorState.TRANSIT;
    } 
    return ActuatorState.ERROR;
  } 

  public void intake() {
    if (actuatorMotorState == ActuatorMotorState.EXTENDING && !beamBreak.isBroken()) {
      rollerState = RollerState.INTAKEING;
      rollerMotor.setControl(dutyCycleRequest.withOutput(IntakeConstants.INTAKE_DUTY_CYCLE));
    }
  }

  public void extake() {
    if (actuatorMotorState == ActuatorMotorState.EXTENDING) {
      rollerState = RollerState.EXTAKEING;
      rollerMotor.setControl(dutyCycleRequest.withOutput(IntakeConstants.EXTAKE_DUTY_CYCLE));
    }
  }

  private void gripp() {
    rollerMotor.setControl(dutyCycleRequest.withOutput(IntakeConstants.INTAKE_GRIP_DUTY_CYCLE));
    rollerState = RollerState.GRIPPING;
  }

  public void stopRoller() {
    if (!beamBreak.isBroken()) {
      rollerState = RollerState.NEUTRAL;
      rollerMotor.setControl(neutralRequest);
    }
  }



  public RollerState getRollerState() {
      return rollerState;
  }

  public void setActuatorMotorState(ActuatorMotorState newState) {
    actuatorMotorState = newState;
    privateSetActuatorMotorState(newState);
  }

  public void extend() {
    setActuatorMotorState(ActuatorMotorState.EXTENDING);
  }

  public void retract() {
    setActuatorMotorState(ActuatorMotorState.RETRACTING);
  }

  private void privateSetActuatorMotorState(ActuatorMotorState newState) {
    switch(newState) {
      case EXTENDING:
        actuatorMotor.setControl(dutyCycleRequest.withOutput(IntakeConstants.ACTUATOR_EXTEND_DUTY_CYCLE));
        break;
      case RETRACTING:
        actuatorMotor.setControl(dutyCycleRequest.withOutput(IntakeConstants.ACTUATOR_RETRACT_DUTY_CYCLE));
        break;
      default:
        actuatorMotor.setControl(neutralRequest);
        break;
      
    }
  }

  @Override
  public void periodic() {

    if (DriverStation.isEnabled()) {
      if (beamBreak.isBroken() && rollerState != RollerState.EXTAKEING) {
        gripp();
      } else if (!beamBreak.isBroken() && actuatorMotorState != ActuatorMotorState.EXTENDING) {
        stopRoller();
      }

      privateSetActuatorMotorState(actuatorMotorState);
    } else {
      stopRoller();
    }
    
  }

  public static enum RollerState {
    INTAKEING,
    EXTAKEING,
    GRIPPING,
    NEUTRAL
  }

  public static enum ActuatorState {
    EXTENDED,
    RETRACTED,
    TRANSIT,
    ERROR
  }

  public static enum ActuatorMotorState {
    /** Motor is actively extending or is holding the intake in an extended state, regardless of limit switch state or if robot is enabled */
    EXTENDING,
    /** Motor is actively retracting or is holding the intake in an retracted state, regardless of limit switch state or if robot is enabled */
    RETRACTING,
    // /** Motor is not active and is not holding the intake in any particular state */
    // NEUTRAL
  }

  public static final class IntakeConstants {
    public static final int ACTUATOR_ID = 0;
    public static final int ROLLER_ID = 0;
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

}
