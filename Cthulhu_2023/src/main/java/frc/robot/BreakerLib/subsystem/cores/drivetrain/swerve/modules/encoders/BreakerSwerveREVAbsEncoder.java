// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Code representation of the REV Through Bore Encoder for use in swerve drive. */
public class BreakerSwerveREVAbsEncoder implements BreakerSwerveAzimuthEncoder {

    private AbsoluteEncoder revEncoder;

    /**
     * Creates a REV absolute encoder connected by roboRIO DIO.
     * 
     * @param port DIO port in use by the encoder.
     */
    public BreakerSwerveREVAbsEncoder(int port) {
    }

    /** Creates a REV absolute encoder connected to a SPARK MAX.
     * 
     * @param spark SPARK MAX the encoder is connected to.
     */
    public BreakerSwerveREVAbsEncoder(CANSparkMax spark) {
        revEncoder = spark.getAbsoluteEncoder(Type.kDutyCycle);
    }

    @Override
    public double getRelative() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getAbsolute() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void config(boolean clockwisePositive, double absoluteOffset) {
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
