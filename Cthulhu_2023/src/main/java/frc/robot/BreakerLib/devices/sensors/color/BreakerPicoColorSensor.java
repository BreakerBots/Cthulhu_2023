// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.devices.sensors.color.BreakerPicoColorSensorLowLevel.RawColor;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** REV Color Sensor V3 implementing the Breaker device interface. */
public class BreakerPicoColorSensor {

  private BreakerPicoColorSensorLowLevel pico;

  /**
   * Create a new BreakerColorSensor.
   */
  public BreakerPicoColorSensor() {
    pico = new BreakerPicoColorSensorLowLevel();
  }

  public BreakerPicoColorSensorInstance createSensor0() {
    return new BreakerPicoColorSensorInstance(pico, true);
  }

  public BreakerPicoColorSensorInstance createSensor1() {
    return new BreakerPicoColorSensorInstance(pico, false);
  }

  public static class BreakerPicoColorSensorInstance extends BreakerGenericColorSensor {
    private BreakerPicoColorSensorLowLevel pico;
    private boolean isSensor0;
    private BreakerPicoColorSensorInstance(BreakerPicoColorSensorLowLevel pico, boolean isSensor0) {
      this.pico = pico;
      this.isSensor0 = isSensor0;
    }

    public RawColor getRawColor() {
      return isSensor0 ? pico.getRawColor0() : pico.getRawColor1();
    }

    @Override
    /** Current color detected by the sensor. */
    public Color getColor() {
      RawColor rc = getRawColor();
      double r = (double) rc.red;
      double g = (double) rc.green;
      double b = (double) rc.blue;
      double mag = r + g + b;
      return new Color(r / mag, g / mag, b / mag);
    }

    @Override
    /** Delivers RGB values plus IR value. */
    public int[] getRawColorsADC() {
      int[] colorVals = new int[4];
      RawColor rc = getRawColor();
      colorVals[0] =  rc.red;
      colorVals[1] =  rc.green;
      colorVals[2] =  rc.blue;
      colorVals[3] =  rc.ir;
      return colorVals;
    }

    @Override
    public double getProximity() {
      return (double) (isSensor0 ? pico.getProximity0() : pico.getProximity1()) / 2048.0;
    }

    @Override
    public void runSelfTest() {
      faultStr = "";
      health = DeviceHealth.NOMINAL;
      if (!(isSensor0 ? pico.isSensor0Connected() : pico.isSensor1Connected())) {
        health = DeviceHealth.INOPERABLE;
        faultStr = isSensor0 ? " COLOR_SENSOR_0_NOT_CONNECTED " : " COLOR_SENSOR_1_NOT_CONNECTED ";
      }
    }

    
    /** 
     * @return boolean
     */
    @Override
    public boolean isUnderAutomaticControl() {
      // TODO Auto-generated method stub
      return false;
    }

    
    /** 
     * @return DevicePowerMode
     */
    @Override
    public DevicePowerMode getPowerMode() {
      // TODO Auto-generated method stub
      return null;
    }

    
    /** 
     * @param managementConfig
     * @param managementPerameters
     * @return DevicePowerMode
     */
    @Override
    public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig, double... managementPerameters) {
      // TODO Auto-generated method stub
      return null;
    }

    
    /** 
     * @param manualPowerMode
     */
    @Override
    public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
      // TODO Auto-generated method stub

    }

    @Override
    public void returnToAutomaticPowerManagement() {
      // TODO Auto-generated method stub

    }
  }

}
