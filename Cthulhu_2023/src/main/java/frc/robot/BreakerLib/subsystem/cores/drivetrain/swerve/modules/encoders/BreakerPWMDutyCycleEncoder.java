// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/**
 * PWM-controlled duty cycle encoders (absolute encoders).
 * 
 * Devices to use include the following:
 * - CTRE Mag Encoder (connected to DIO)
 * - Rev Through Bore
 * - US Digital MA3 and other simple PWM encoders.
 */
public class BreakerPWMDutyCycleEncoder implements BreakerSwerveAzimuthEncoder {

    private DutyCycleEncoder dcEncoder;
    private double offset = 0;
    private int invertSign = 1;

    public BreakerPWMDutyCycleEncoder(int channel) {
        dcEncoder = new DutyCycleEncoder(channel);
    }

    @Override
    public double getRelative() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getAbsolute() {
        return invertSign * BreakerMath.angleModulus(dcEncoder.get() * 360 + offset);
    }

    @Override
    public void config(boolean clockwisePositive, double offset) {
        invertSign = clockwisePositive ? 1 : -1;
        this.offset = offset;
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Class<?> getBaseEncoderType() {
        return dcEncoder.getClass();
    }

    @Override
    public Object getBaseEncoder() {
        return dcEncoder;
    }
}
