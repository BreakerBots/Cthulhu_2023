// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.factory.BreakerProCANCoderFactory;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

/** Add your docs here. */
public class BreakerSwerveProCANcoder implements BreakerSwerveAzimuthEncoder {
    private CANcoder encoder;
    public BreakerSwerveProCANcoder(CANcoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public double getRelative() {
        return encoder.getPosition().getValue() * 360.0;
    }

    @Override
    public double getAbsolute() {
        return encoder.getAbsolutePosition().getValue() * 360.0;
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        return BreakerPhoenix6Util.checkCANcoderFaultsAndConnection(encoder);
    }

    @Override
    public Class<?> getBaseEncoderType() {
        return CANcoder.class;
    }

    @Override
    public Object getBaseEncoder() {
        return encoder;
    }

    @Override
    public void config(boolean invertEncoder, double absoluteOffset) {
        BreakerProCANCoderFactory.configExistingCANCoder(encoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, absoluteOffset / 360.0, SensorDirectionValue.CounterClockwise_Positive);
    }

}