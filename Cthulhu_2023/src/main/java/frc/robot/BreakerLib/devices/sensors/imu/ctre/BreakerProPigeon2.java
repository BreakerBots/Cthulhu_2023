// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.BreakerLib.devices.sensors.imu.ctre;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenixpro.Timestamp.TimestampSource;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.devices.sensors.BreakerGenericMagnetometer;
import frc.robot.BreakerLib.devices.sensors.imu.BreakerGenericIMU;
import frc.robot.BreakerLib.physics.vector.BreakerVector3;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/* CTRE Pigeon 2 implementing the Breaker device interface and Breaker IMU interface,  */
public class BreakerProPigeon2 extends BreakerGenericIMU implements BreakerGenericMagnetometer {
  private Pigeon2 pigeon;

  /** Creates a new PigeonIMU 2 object. */
  public BreakerProPigeon2(int deviceID) {
    pigeon = new Pigeon2(deviceID);
    deviceName = "Pigeon2_IMU (" + deviceID + ") ";
  }

  /** Creates a new PigeonIMU 2 object. */
  public BreakerProPigeon2(int deviceID, String busName) {
    pigeon = new Pigeon2(deviceID, busName);
    deviceName = "Pigeon2_IMU (" + deviceID + ") ";
  }

  @Override
  public double getPitchDegrees() {
    return BreakerMath.angleModulus(pigeon.getPitch().getValue());
  }

  @Override
  public double getYawDegrees() {
    return BreakerMath.angleModulus(pigeon.getAngle());
  }

  @Override
  public double getRollDegrees() {
    return BreakerMath.angleModulus(pigeon.getRoll().getValue());
  }

  @Override
  public Rotation2d getPitchRotation2d() {
    return Rotation2d.fromDegrees(getPitchDegrees());
  }

  @Override
  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(getYawDegrees());
  }

  @Override
  public Rotation2d getRollRotation2d() {
    return Rotation2d.fromDegrees(getRollDegrees());
  }

  @Override
  public Rotation3d getRotation3d() {
    return new Rotation3d(getQuaternion());
  }

  @Override
  public double[] getRawAngles() {
    return new double[]{pigeon.getYaw().getValue(), pigeon.getPitch().getValue(), pigeon.getRoll().getValue()};
  }

  @Override
  public double getRawYaw() {
    return getRawAngles()[0];
  }

  @Override
  public double getRawPitch() {
    return getRawAngles()[1];
  }

  @Override
  public double getRawRoll() {
    return getRawAngles()[2];
  }

  /** Does nothing. */
  @Override
  public void setPitch(double value) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setYaw(double value) {
    pigeon.setYaw(0);
  }

  /** Does nothing. */
  @Override
  public void setRoll(double value) {
  }

  /** Sets yaw to 0 */
  @Override
  public void reset() {
    pigeon.setYaw(0);
  }

  public double[] getRawGyroRates() {
    return new double[] {pigeon.getAngularVelocityX().getValue(), pigeon.getAngularVelocityY().getValue(), pigeon.getAngularVelocityZ().getValue()};
  }

  @Override
  public double getRawPitchRate() {
    return getRawGyroRates()[0];
  }

  @Override
  public double getRawRollRate() {
    return getRawGyroRates()[1];
  }

  @Override
  public double getRawYawRate() {
    return getRawGyroRates()[2];
  }

  @Override
  public double getPitchRate() {
    return getRawPitchRate();
  }

  @Override
  public double getYawRate() {
    return getRawYawRate();
  }

  @Override
  public double getRollRate() {
    return getRawRollRate();
  }

  @Override
  public double[] getRawAccelerometerVals() {
    return new double[] {pigeon.getAccelerationX().getValue(), pigeon.getAccelerationY().getValue(), pigeon.getAccelerationZ().getValue()};
  }

  @Override
  /** @return Unbiased accelerometer x-value in G. */
  public double getRawAccelX() {
    return pigeon.getAccelerationX().getValue();
  }

  @Override
  /** @return Unbiased accelerometer y-value in G. */
  public double getRawAccelY() {
    return pigeon.getAccelerationY().getValue();
  }

  @Override
  /** @return Unbiased accelerometer z-value in G. */
  public double getRawAccelZ() {
    return pigeon.getAccelerationZ().getValue();
  }

  /** @return Pigeon's runtime in seconds (max of 255) */
  public int getPigeonUpTime() {
    return pigeon.getUpTime().getValue().intValue();
  }

  public BreakerVector3 getGravityVector() {
    return new BreakerVector3(pigeon.getGravityVectorX().getValue(), pigeon.getGravityVectorY().getValue(), pigeon.getGravityVectorZ().getValue());
  }

  public double[] getBiasedAccelerometerVals() {
    BreakerVector3 vec = new BreakerVector3(getRawAccelX(), getRawAccelY(), getRawAccelZ()).minus(getGravityVector());
    return new double[] {vec.getMagnitudeX(), vec.getMagnitudeY(), vec.getMagnatudeZ()};
  }

  /** @return Biased accelerometer x-value in G. */
  public double getBaisedAccelX() {
    return getBiasedAccelerometerVals()[0];
  }

  /** @return Biased accelerometer y-value in G. */
  public double getBaisedAccelY() {
    return getBiasedAccelerometerVals()[1];
  }

  /** @return Biased accelerometer z-value in G. */
  public double getBiasedAccelZ() {
    return getBiasedAccelerometerVals()[2];
  }

  @Override
  public Rotation3d getRawRotation3d() {
    return new Rotation3d(Math.toRadians(getRawAngles()[2]), Math.toRadians(getRawAngles()[1]),
        Math.toRadians(getRawAngles()[0]));
  }

  @Override
  public void runSelfTest() {
    faultStr = "";
    health = DeviceHealth.NOMINAL;
    if (pigeon.getFault_Hardware().getValue()) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " hardware_fault ";
    }
    if (pigeon.getFault_BootupMagnetometer().getValue()) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " mag_fault ";
    }
    if (pigeon.getFault_BootupGyroscope().getValue()) {
      health = DeviceHealth.INOPERABLE;
      faultStr += "  gyro_fault ";
    }
    if (pigeon.getFault_BootupAccelerometer().getValue()) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " accel_fault ";
    }
    if (pigeon.getFault_Undervoltage().getValue()) {
      health = (health != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : health;
      faultStr += " under_6.5V ";
    }
    if (pigeon.getVersion().getValue() == 0) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " device_disconnected ";
    }
  }

  @Override
  public double[] getRawFieldStrengths() {
    return new double[] {pigeon.getRawMagneticFieldX().getValue(), pigeon.getRawMagneticFieldY().getValue(), pigeon.getRawMagneticFieldZ().getValue()};
  }

  @Override
  public double[] getBiasedFieldStrengths() {
    return new double[] {pigeon.getMagneticFieldX().getValue(), pigeon.getMagneticFieldY().getValue(), pigeon.getMagneticFieldZ().getValue()};

  }

  @Override
  /** not supported by phoenix pro */
  public double getCompassFieldStrength() {
    return 0.0;
  }

  @Override
  /** not supported by phoenix pro */
  public double getCompassHeading() {
    return 0.0;
  }

  @Override
  /** not supported by phoenix pro */
  public double getRawCompassHeading() {
    return 0.0;
  }

  @Override
  public Quaternion getQuaternion() {
    return new Quaternion(pigeon.getQuatW().getValue(), pigeon.getQuatX().getValue(), pigeon.getQuatY().getValue(), pigeon.getQuatZ().getValue());
  }

  /**
   * Does nothing. Calibration is done on boot and can be performed with Phoenix
   * Tuner.
   */
  public void calibrate() {
    pigeon.calibrate();
  }

}
