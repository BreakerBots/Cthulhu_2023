// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.yousif;

import java.util.Objects;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class BreakerArmJoint extends TrapezoidProfileSubsystem {

  private WPI_TalonFX motor;
  private WPI_CANCoder encoder;
  private ArmFeedforward armFF;
  private TrapezoidProfile.Constraints constraints;

  public BreakerArmJoint(WPI_TalonFX motor, TrapezoidProfile.Constraints constraints, WPI_CANCoder encoder) {
    super(constraints, Rotation2d.fromDegrees(encoder.getPosition()).getRadians());
    this.motor = motor;
    this.encoder = encoder;
  }

  // TODO: Brainstorm non-CANCoder support
  public BreakerArmJoint(WPI_TalonFX motor, TrapezoidProfile.Constraints constraints) {
    this(motor, constraints, null);
  }

  public BreakerArmJoint setPID(double kP, double kI, double kD) {
    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_kD(0, kD);
    motor.selectProfileSlot(0, 0);
    return this;
  }

  public BreakerArmJoint setFeedForward(double kS, double kV, double kA, double kG) {
    armFF = new ArmFeedforward(kS, kG, kV, kA);
    return this;
  }

  /** @return Absolute angle returned by the CANCoder. */
  public double getJointAngle() {
    return encoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
  }

  @Override
  protected void useState(State state) {
    // TODO Auto-generated method stub
    
  }
}
