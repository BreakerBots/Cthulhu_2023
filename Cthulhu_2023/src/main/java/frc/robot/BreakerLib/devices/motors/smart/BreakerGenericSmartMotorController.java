// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.smart;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.BreakerLib.devices.BreakerGenericDeviceBase;

/** Add your docs here. */
public abstract class BreakerGenericSmartMotorController extends BreakerGenericDeviceBase implements MotorController {


    public static enum SmartMotorControlMode {
        PRECENT_OUTPUT,
        VOLTAGE,
        CURRENT,
        POSITION,
        VELOCITY
    }

    public static enum SmartMotorDemandType {
        NONE,
        PRECENT_OUTPUT,
        VOLTAGE
    }
    private MotorController baseMotorController;
    public BreakerGenericSmartMotorController(MotorController baseMotorController) {
        this.baseMotorController = baseMotorController;
    }

    public abstract void set(SmartMotorControlMode mode, double demand0, SmartMotorDemandType ffType, double demand1);

    public void set(SmartMotorControlMode mode, double demand) {
        set(mode, demand, SmartMotorDemandType.NONE, 0.0);
    }

    @Override
    /**
   * Common interface for setting the speed of a motor controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
    public void set(double speed) {
        set(SmartMotorControlMode.PRECENT_OUTPUT, speed);
    }

    @Override
    public double get() {
        return baseMotorController.get();
    }

    @Override
    /**
     * Common interface for inverting direction of a motor controller.
     *
     * @param isInverted The state of inversion true is inverted.
     */
    public void setInverted(boolean isInverted) {
        baseMotorController.setInverted(isInverted);
    }

  @Override
  /**
   * Common interface for returning if a motor controller is in the inverted state or not.
   *
   * @return isInverted The state of the inversion true is inverted.
   */
  public boolean getInverted() {
    return baseMotorController.getInverted();
  }

  @Override
  /** Disable the motor controller. */
  public void disable() {
    baseMotorController.disable();
  }

  @Override
  /**
   * Stops motor movement. Motor can be moved again by calling set without having to re-enable the
   * motor.
   */
  public void stopMotor() {
    baseMotorController.stopMotor();
  }

  public abstract void configPIDSlot(int slot, BreakerSmartMotorControllerConfig config);

  public abstract void selectPIDSlot(int slot);

    
}
