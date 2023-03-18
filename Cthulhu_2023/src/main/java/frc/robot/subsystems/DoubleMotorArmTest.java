// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;

public class DoubleMotorArmTest extends SubsystemBase {
  /** Creates a new NewArmTest. */

  private WPI_TalonFX motor0, motor1;

  private BreakerXboxController controller;

  public DoubleMotorArmTest(BreakerXboxController controller) {
    this.controller = controller;
    motor0 = new WPI_TalonFX(41);
    motor1 = new WPI_TalonFX(40);
    motor0.setNeutralMode(NeutralMode.Brake);
    motor0.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    motor1.setNeutralMode(NeutralMode.Brake);
    motor1.setInverted(TalonFXInvertType.Clockwise);
    motor0.setInverted(TalonFXInvertType.Clockwise);
    motor1.follow(motor0);
  }

  @Override
  public void periodic() {
    motor0.set(controller.getRightTrigger().get() - controller.getLeftTrigger().get());
  }
}
