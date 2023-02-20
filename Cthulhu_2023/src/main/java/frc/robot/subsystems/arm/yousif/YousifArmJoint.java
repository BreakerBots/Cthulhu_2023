// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.yousif;

import org.opencv.core.RotatedRect;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;

public class YousifArmJoint extends TrapezoidProfileSubsystem {

  private WPI_TalonFX motor;
  private WPI_CANCoder encoder;
  private ArmFeedforward armFF;
  private TrapezoidProfile.Constraints constraints;

  public YousifArmJoint(WPI_TalonFX motor, TrapezoidProfile.Constraints constraints, WPI_CANCoder encoder,
      double encoderOffset) {
    super(constraints, Rotation2d.fromDegrees(encoder.getPosition()).getRadians());
    this.motor = motor;
    this.encoder = encoder;
    this.constraints = constraints;
    BreakerCANCoderFactory.configExistingCANCoder(encoder, SensorInitializationStrategy.BootToAbsolutePosition,
        AbsoluteSensorRange.Signed_PlusMinus180, encoderOffset, false);
    this.enable();
  }

  // TODO: Brainstorm non-CANCoder support
  public YousifArmJoint(WPI_TalonFX motor, TrapezoidProfile.Constraints constraints) {
    this(motor, constraints, null, 0);
  }

  public YousifArmJoint setPID(double kP, double kI, double kD) {
    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_kD(0, kD);
    motor.selectProfileSlot(0, 0);
    return this;
  }

  public YousifArmJoint setFeedForward(double kS, double kG, double kV, double kA) {
    armFF = new ArmFeedforward(kS, kG, kV, kA);
    return this;
  }

  /** @return Absolute angle returned by the CANCoder. */
  public double getJointAngle() {
    return encoder.getAbsolutePosition();
  }

  public Rotation2d getJointAngleRot2d() {
    return Rotation2d.fromDegrees(getJointAngle());
  }

  @Override
  protected void useState(TrapezoidProfile.State setpoint) {
    double ffComponent = armFF.calculate(setpoint.position, setpoint.velocity);
    double goalRadians = setpoint.position - getJointAngleRot2d().getRadians();
    motor.set(TalonFXControlMode.Position, BreakerCTREUtil.radiansToCANCoderNativeUnits(goalRadians),
        DemandType.ArbitraryFeedForward, ffComponent/ RobotController.getBatteryVoltage());
  }
}
