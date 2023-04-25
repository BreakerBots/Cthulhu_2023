// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.vendorutil;

import java.util.HashMap;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.hardware.core.CoreTalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public class BreakerPhoenixProUtil {

    /**
   * Logs an error to BreakerLog if designated error is discovered.
   * 
   * @param statusCode  CTRE error code to detect.
   * @param message Message to log when the error is detected.
   */
  public static void checkStatusCode(StatusCode statusCode, String message) {
    if (statusCode != StatusCode.OK) {
      BreakerLog.logError(statusCode + " - " + message);
    }
  }


  public static void setBrakeMode(TalonFX motor, boolean isEnabled) {
    MotorOutputConfigs moc = new MotorOutputConfigs();
    checkStatusCode(motor.getConfigurator().refresh(moc), "Failded to refresh TalonFX motor config");
    moc.NeutralMode = isEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    checkStatusCode(motor.getConfigurator().apply(moc), "Failded to apply TalonFX brake mode motor config");
  }

  /**
   * @param motor
   * @return Pair<DeviceHealth, String>
   */
  public static Pair<DeviceHealth, String> checkMotorFaultsAndConnection(CoreTalonFX motor) {
    Pair<DeviceHealth, String> pair = getMotorHealthAndFaults(motor.getFaultField().getValue());
    String retStr = pair.getSecond();
    DeviceHealth retHealth = pair.getFirst();
    if (motor.getVersion().getValue() == 0) {
      retStr += " device_disconnected ";
      retHealth = DeviceHealth.INOPERABLE;
    }
    return new Pair<DeviceHealth, String>(retHealth, retStr);
  }

   /**
   * Get CTRE motor controller faults and device health.
   * 
   * @param motorFaults Motor controller faults.
   * @return Motor controller device health and error type (if any).
   */
  public static Pair<DeviceHealth, String> getMotorHealthAndFaults(int bitField) {
    HashMap<Integer, Pair<DeviceHealth, String>> map = new HashMap<>();
    map.put(0, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_under_6.5v "));
    // map.put(1, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    // map.put(2, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    // map.put(3, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    // map.put(4, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    map.put(5, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " hardware_failure "));
    map.put(6, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_activated_or_reset_while_robot_on "));
    map.put(9, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_activated_or_reset_while_robot_on "));
    map.put(7, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " sensor_overflow "));
    map.put(8, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " sensor_out_of_phase "));
    map.put(10, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " remote_sensor_not_detected "));
    map.put(11, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " API_or_firmware_error "));
    map.put(12, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " supply_voltage_above_rated_max "));
    map.put(13, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " unstable_supply_voltage "));
    return BreakerVendorUtil.getDeviceHealthAndFaults(bitField, map);
  }
}
