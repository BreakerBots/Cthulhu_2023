// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.smart;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestable;

/** Add your docs here. */
public interface BreakerGenericSmartMotorController extends BreakerSelfTestable, MotorController {


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

    public static enum SmartMotorNeutralMode { 
      COAST,
      BRAKE
    }

    public abstract void set(SmartMotorControlMode mode, double demand0, SmartMotorDemandType ffType, double demand1);

    public default void set(SmartMotorControlMode mode, double demand) {
        set(mode, demand, SmartMotorDemandType.NONE, 0.0);
    }
    
    public abstract void setNeutralMode(NeutralMode neutralMode);

    public abstract NeutralMode getNeutralMode();
}
