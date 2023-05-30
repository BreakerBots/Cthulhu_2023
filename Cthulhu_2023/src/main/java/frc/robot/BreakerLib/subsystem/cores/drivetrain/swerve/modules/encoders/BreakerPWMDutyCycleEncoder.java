// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public class BreakerPWMDutyCycleEncoder implements BreakerSwerveAzimuthEncoder {

    DutyCycleEncoder dcEncoder;

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
        return dcEncoder.get();
    }

    @Override
    public void config(boolean clockwisePosative, double absoluteOffset) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Class<?> getBaseEncoderType() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Object getBaseEncoder() {
        // TODO Auto-generated method stub
        return null;
    }}
