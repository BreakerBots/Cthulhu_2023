// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.simplearm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;

import static frc.robot.Constants.ArmConstants.*;
public class SimpleArm extends SubsystemBase {
  /** Creates a new SimpleArm. */
  private WPI_TalonFX proximalMotor, distalMotor;
  private BreakerXboxController controller;
  public SimpleArm(BreakerXboxController controller) {
    this.controller = controller;
    proximalMotor = new WPI_TalonFX(PROXIMAL_MOTOR_ID);
    distalMotor = new WPI_TalonFX(DISTAL_MOTOR_ID);
  }
    

  @Override
  public void periodic() {
    distalMotor.set(controller.getLeftThumbstick().getY() * 0.15);
    proximalMotor.set(controller.getRightThumbstick().getY() * 0.15);
  }
}
