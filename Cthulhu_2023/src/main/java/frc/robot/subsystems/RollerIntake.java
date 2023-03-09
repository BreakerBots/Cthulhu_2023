// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.RollerIntakeConstants.*;

public class RollerIntake extends SubsystemBase {
  private WPI_TalonSRX motor;
  private boolean isConeModeSelected = true;

  /** Creates a new RollerIntake. */
  public RollerIntake() {
    this.motor = new WPI_TalonSRX(INTAKE_ID);
    motor.configPeakCurrentLimit(40);
    motor.configPeakCurrentDuration(1000);
  }

  @Override
  public void periodic() {
    var amps = motor.getStatorCurrent();
    SmartDashboard.putNumber("AMPS", amps);
  }

  public boolean isConeModeSelected() {
      return isConeModeSelected;
  }

  public void 
  toggleConeModeSelected() {
      isConeModeSelected = !isConeModeSelected;
  }

  public void runSelectedIntakeMode() {
    if (isConeModeSelected) {
      runConeIntake();
    } else {
      runCubeIntake();
    }
  }

  public void runConeIntake() {
    motor.set(0.80);
  }

  public void runCubeIntake() {
    motor.set(0.2);
  }

  public void eject() {
    motor.set(-1);
  }

  public void stop() {
    motor.stopMotor();
  }
}
