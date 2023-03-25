// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.MiscConstants.*;

public class SebArm extends SubsystemBase {
  public enum State {
    STOW(new Rotation2d()),
    LOW(new Rotation2d()),
    PICKUPHIGH(new Rotation2d()),
    PLACEMID(new Rotation2d()),
    UNKNOWN(new Rotation2d());

    public final Rotation2d rot;

    private State(Rotation2d rot) {
      this.rot = rot;
    }
  }

  /** Creates a new NewArmTest. */

  // Motor reverse moves arm to intake position. Motor forward moves arm to stow position.
  

  private WPI_TalonFX motor0, motor1;
  private Rotation2d desiredRot;
  private WPI_CANCoder canCoder;
  private PIDController pid;

  private BreakerXboxController controller;

  /**
   * Arm class for the new arm.
   * 
   * @param controller
   */
  public SebArm(BreakerXboxController controller) {
    this.controller = controller;
    motor0 = new WPI_TalonFX(MAIN_MOTOR_ID, CANIVORE_1);
    motor1 = new WPI_TalonFX(SUB_MOTOR_ID, CANIVORE_1);
    canCoder = new WPI_CANCoder(ARM_CANCODER_ID);
    pid = new PIDController(0, 0, 0); // TODO: Set actual PID values for this constructor.

    motor0.setNeutralMode(NeutralMode.Brake);
    motor0.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    motor0.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    motor1.setStatusFramePeriod(0, 0, 0);
    motor1.setNeutralMode(NeutralMode.Brake);
    motor0.setInverted(TalonFXInvertType.CounterClockwise);
    motor1.follow(motor0);
    motor1.setInverted(TalonFXInvertType.FollowMaster);
  }

  @Override
  public void periodic() {
    double ctrlInput = controller.getRightTrigger().get() - controller.getLeftTrigger().get();
    motor0.set(-ctrlInput);
  }
}
