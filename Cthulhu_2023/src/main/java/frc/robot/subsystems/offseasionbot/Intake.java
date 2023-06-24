// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.offseasionbot;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.BreakerBeamBreak;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX actuatorMotor;
  private final TalonFX rollerMotor;
  private final BreakerBeamBreak beamBreak;

  private final DutyCycleOut dutyCycleRequest;
  private final NeutralOut neutralRequest;

  private RollerState rollerState;
  private final Supplier<ForwardLimitValue> forwardLimit;
  private final Supplier<ReverseLimitValue> reverseLimit;
  private boolean actuatorExtended;
  public Intake() {
    actuatorMotor = new TalonFX(0, "placeholder");
    rollerMotor = new TalonFX(0, "placeholder");
    beamBreak = new BreakerBeamBreak(0, true);

    forwardLimit = actuatorMotor.getForwardLimit().asSupplier();
    reverseLimit = actuatorMotor.getReverseLimit().asSupplier();
    rollerState = RollerState.INACTIVE;
    actuatorExtended = reverseLimit.get() != ReverseLimitValue.ClosedToGround;

    dutyCycleRequest = new DutyCycleOut(0, false, false);
    neutralRequest = new NeutralOut();
  }

  public void setRollerIntakeing() {
    if (actuatorExtended && !beamBreak.isBroken()) {
      rollerState = RollerState.INTAKEING;
    }
  }

  public void setRollerExtakeing() {
    if (actuatorExtended) {
      rollerState = RollerState.EXTAKEING;
      rollerMotor.setControl(dutyCycleRequest.withOutput(IntakeConstants.EXTAKE_DUTY_CYCLE));
    }
  }

  private void setRollerGripping() {
    rollerMotor.setControl(dutyCycleRequest.withOutput(IntakeConstants.INTAKE_GRIP_DUTY_CYCLE));
    rollerState = RollerState.GRIPPING;
  }

  public void stopRoller() {
    if (!beamBreak.isBroken()) {
      rollerState = RollerState.INACTIVE;
      rollerMotor.setControl(neutralRequest);
    }
  }

  public RollerState getRollerState() {
      return rollerState;
  }

  public void extend() {
    actuatorMotor.setControl(dutyCycleRequest.withOutput(IntakeConstants.ACTUATOR_EXTEND_DUTY_CYCLE));
    actuatorExtended = true;
  }

  public void retract() {
    actuatorMotor.setControl(dutyCycleRequest.withOutput(IntakeConstants.ACTUATOR_RETRACT_DUTY_CYCLE));
    actuatorExtended = false;
  }


  @Override
  public void periodic() {

    if (beamBreak.isBroken() && rollerState != RollerState.EXTAKEING) {
      setRollerGripping();
    } else if (!beamBreak.isBroken() && !actuatorExtended) {
      stopRoller();
    }
    
  }

  public static enum RollerState {
    INTAKEING,
    EXTAKEING,
    GRIPPING,
    INACTIVE
  }

  public static final class IntakeConstants {
    public static final double ACTUATOR_EXTEND_DUTY_CYCLE = 0;
    public static final double ACTUATOR_RETRACT_DUTY_CYCLE = 0;

    public static final double INTAKE_DUTY_CYCLE = 0;
    public static final double INTAKE_GRIP_DUTY_CYCLE = 0;
    public static final double EXTAKE_DUTY_CYCLE = 0;

  }

}
