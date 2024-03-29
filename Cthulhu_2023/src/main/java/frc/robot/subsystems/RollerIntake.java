// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.test.selftest.SystemDiagnostics;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;

public class RollerIntake extends SubsystemBase{

  private WPI_TalonSRX motor = new WPI_TalonSRX(Constants.RollerIntakeConstants.INTAKE_ID);

  /** Creates a new Intake. */
  public RollerIntake() {
    SystemDiagnostics diag = new SystemDiagnostics("RollerIntake");
    diag.addPhoenix5MotorController(motor);

    BreakerPhoenix5Util.checkError(motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15.0, 15.0, 1.5)),
            " Failed to config swerve module turn motor ");
  }

  public InstantCommand ejectCmd() {
    return new InstantCommand(this::eject);
  }

  public InstantCommand stopCmd() {
    return new InstantCommand(this::stop);
  }

  public InstantCommand startCmd() {
    return new InstantCommand(this::start);
  }

  @Override
  public void periodic() {}

  public void start() {
    motor.set(-0.4);
  }

  public void eject() {
    motor.set(1.0);
  }

  public void stop() {
    motor.stopMotor();
  }
}
