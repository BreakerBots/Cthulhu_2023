        // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.factory;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;

import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenixProUtil;

/** Factory for producing CANcoders. */
public class BreakerProCANCoderFactory {

    /**
     * Creates CANCoder on default bus.
     * 
     * @param deviceID CAN ID
     * @param initializationStrategy Boot to zero or boot to absolute position.
     * @param absoluteSensorRange Unsigned 0 to 360 deg or signed +-180 deg.
     * @param absoluteOffsetDegrees Offset of encoder in degrees.
     * @param encoderDirection False = counterclockwise rotation is positive (facing towards LED).
     * 
     * @return CANCoder with desired settings.
     */
    public static CANcoder createCANCoder(int deviceID, SensorInitializationStrategy initializationStrategy,
            AbsoluteSensorRange absoluteSensorRange, double absoluteOffsetDegrees, boolean encoderDirection) {
        CANCoder encoder = new CANCoder(deviceID);
        configExistingCANCoder(encoder, initializationStrategy, absoluteSensorRange, absoluteOffsetDegrees,
                encoderDirection);
        return encoder;
    }

    /**
     * Creates CANCoder on specified bus.
     * 
     * @param deviceID CAN ID
     * @param busName CANivore device name/serial # or "rio" for RoboRIO bus.
     * 
     * @return CANCoder with desired settings.
     */
    public static WPI_CANCoder createCANCoder(int deviceID, String busName,
            SensorInitializationStrategy initializationStrategy, AbsoluteSensorRange absoluteSensorRange,
            double absoluteOffsetDegrees, boolean encoderDirection) {
        CANCoder encoder = new CANCoder(deviceID, busName);
        configExistingCANCoder(encoder, initializationStrategy, absoluteSensorRange, absoluteOffsetDegrees,
                encoderDirection);
        return encoder;
    }

    /**
     * Configure pre-existing CANCoder.
     * 
     * @param encoder CANCoder to config.
     * @param initializationStrategy Boot to zero or boot to absolute position.
     * @param absoluteSensorRange Unsigned 0 to 1 rotation or signed +-0.5 rotations.
     * @param absoluteOffsetRotations Offset of encoder as a fraction of a rotation (1 to -1)
     * @param encoderDirection False = counterclockwise rotation is positive (facing towards LED).
     */
    public static void configExistingCANCoder(CANcoder encoder, AbsoluteSensorRangeValue absoluteSensorRange, double absoluteOffsetRotations, boolean encoderDirection) {
               CANcoderConfiguration config =  new CANcoderConfiguration();
               config.MagnetSensor.AbsoluteSensorRange = absoluteSensorRange;
               config.MagnetSensor.MagnetOffset = absoluteOffsetRotations;
               config.MagnetSensor.SensorDirection = encoderDirection;
                encoder.getConfigurator().apply(config)
                encoder.getAbsolutePosition()

        BreakerPhoenixProUtil.checkStatusCode(encoder.getConfigurator().apply(config), null);
    }
}
