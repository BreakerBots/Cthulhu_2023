// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;


import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;

/** Add your docs here. */
public class BreakerSwerveCANcoder implements BreakerSwerveAzimuthEncoder {
    private WPI_CANCoder encoder;
    public BreakerSwerveCANcoder(WPI_CANCoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public double getRelative() {
        return encoder.getPosition();
    }

    @Override
    public double getAbsolute() {
        return encoder.getAbsolutePosition();
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        return BreakerCTREUtil.checkCANCoderFaultsAndConnection(encoder);
    }

    @Override
    public Class<?> getBaseEncoderType() {
        return WPI_CANCoder.class;
    }

    @Override
    public Object getBaseEncoder() {
        return encoder;
    }

    @Override
    public void config(boolean invertEncoder, double absoluteOffset) {
        BreakerCANCoderFactory.configExistingCANCoder(encoder, SensorInitializationStrategy.BootToAbsolutePosition,
        AbsoluteSensorRange.Signed_PlusMinus180, absoluteOffset, invertEncoder);
    }

}