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
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;

public class RollerIntake extends SubsystemBase{

  public InstantCommand ejectCmd = new InstantCommand(this::eject);
  public InstantCommand stopCmd = new InstantCommand(this::stop);
  public InstantCommand startCmd = new InstantCommand(this::start);

  private WPI_TalonSRX motor = new WPI_TalonSRX(Constants.RollerIntakeConstants.INTAKE_ID);

  /** Creates a new Intake. */
  public RollerIntake() {
    SystemDiagnostics diag = new SystemDiagnostics("RollerIntake");
    diag.addCTREMotorController(motor);

    BreakerCTREUtil.checkError(motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15.0, 15.0, 1.5)),
            " Failed to config swerve module turn motor ");
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
