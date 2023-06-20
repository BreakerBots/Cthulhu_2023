// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/**
 * Code representation of the REV Through Bore Encoder for use in swerve drive.
 */
public class BreakerSwerveSparkDutyCycleEncoder implements BreakerSwerveAzimuthEncoder {

    private AbsoluteEncoder revEncoder;

    /**
     * Creates a REV absolute encoder connected to a SPARK MAX.
     * 
     * @param spark SPARK MAX the encoder is connected to.
     */
    public BreakerSwerveSparkDutyCycleEncoder(CANSparkMax spark, int averageSamplingBitDepth) {
        revEncoder = spark.getAbsoluteEncoder(Type.kDutyCycle);
        revEncoder.setPositionConversionFactor(360);
        revEncoder.setAverageDepth(averageSamplingBitDepth);
    }

    @Override
    public double getRelative() {
        return revEncoder.getPosition();
    }

    @Override
    public double getAbsolute() {
        return revEncoder.getPosition();
    }

    @Override
    public void config(boolean invertEncoder, double offset) {
        revEncoder.setInverted(invertEncoder);
        revEncoder.setZeroOffset(offset);
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        return new Pair<DeviceHealth,String>(DeviceHealth.NOMINAL, "");
    }

    @Override
    public Class<?> getBaseEncoderType() {
        return AbsoluteEncoder.class;
    }

    @Override
    public Object getBaseEncoder() {
        return revEncoder;
    }
}
