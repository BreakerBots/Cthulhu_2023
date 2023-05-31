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
    private double offset = 0;
    private int invertSign = 1;

    /**
     * Creates a REV absolute encoder connected to a SPARK MAX.
     * 
     * @param spark SPARK MAX the encoder is connected to.
     */
    public BreakerSwerveSparkDutyCycleEncoder(CANSparkMax spark) {
        revEncoder = spark.getAbsoluteEncoder(Type.kDutyCycle);
    }

    @Override
    public double getRelative() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getAbsolute() {
        return invertSign * BreakerMath.angleModulus(revEncoder.getPosition() * 360 + offset);
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
        return AbsoluteEncoder.class;
    }

    @Override
    public Object getBaseEncoder() {
        return revEncoder;
    }
}
