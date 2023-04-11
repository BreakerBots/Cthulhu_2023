// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.imu;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Fuses multiple IMUs together through a weighted average to produce steadier results. */
public class BreakerFusedIMU extends BreakerGenericIMU{

    List<BreakerGenericIMU> imuList;
    List<Double> weightList;

    public BreakerFusedIMU(Pair<BreakerGenericIMU, Double>... imuWeightPairs) {
        int pairArrLen = imuWeightPairs.length;

        for (int i = 0; i < pairArrLen; i++) {
            imuList.add(imuWeightPairs[i].getFirst());
            weightList.add(imuWeightPairs[i].getSecond());
        }
    }

    @Override
    public double getPitchDegrees() {
        return 0;
    }

    @Override
    public double getRollDegrees() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Rotation2d getPitchRotation2d() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Rotation2d getRollRotation2d() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Quaternion getQuaternion() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Rotation3d getRotation3d() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double[] getRawAngles() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getRawPitch() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRawRoll() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setPitch(double value) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setRoll(double value) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double[] getRawGyroRates() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getRawPitchRate() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRawRollRate() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getPitchRate() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRollRate() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Rotation3d getRawRotation3d() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getYawDegrees() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Rotation2d getYawRotation2d() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void setYaw(double value) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getYawRate() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void calibrate() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getRawYaw() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRawYawRate() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double[] getRawAccelerometerVals() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getRawAccelX() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRawAccelY() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRawAccelZ() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void runSelfTest() {
        // TODO Auto-generated method stub
        
    }}
