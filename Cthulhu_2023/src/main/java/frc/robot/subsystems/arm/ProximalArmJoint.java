// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;
import frc.robot.BreakerLib.util.math.BreakerMath;

public class ProximalArmJoint extends SubsystemBase {
  WPI_CANCoder proxEncoder;
  WPI_TalonFX proxMotor; // + is forward, - is backward.
  PIDController pid;
  double proxTgt;

  /** Creates a new ArmDo. */
  public ProximalArmJoint(WPI_TalonFX proxMotor, WPI_CANCoder proxEncoder) {
    this.proxMotor = proxMotor;
    this.proxEncoder = proxEncoder;
    this.pid = new PIDController(PROX_KP, PROX_KI, PROX_KD);
    pid.setTolerance(5, 2);
    proxTgt = proxEncoder.getAbsolutePosition();
  }

  public Rotation2d getJointAngle() {
    return Rotation2d.fromDegrees(proxEncoder.getAbsolutePosition());
  }

  public double getJointVel() {
    return Math.toRadians(proxEncoder.getVelocity());
  }

  public void setTarget(Rotation2d tgt) {
      this.proxTgt = tgt.getDegrees();
  }

  public double getRawMotorOut() {
    return proxMotor.get();
  }

  @Override
  public void periodic() {
    if (!BreakerMath.epsilonEquals(getJointAngle().getDegrees(), proxTgt, 5)) {
      proxMotor.set(MathUtil.clamp(-pid.calculate(proxEncoder.getAbsolutePosition(), proxTgt), -.35, .35));
    }
  }
}
