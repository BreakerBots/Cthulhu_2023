// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.vendorutil;

import java.util.ArrayList;
import java.util.HashMap;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public class BreakerPhoenix6Util {

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


  public static void setBrakeMode(boolean isEnabled, CoreTalonFX... motors) {
    for (CoreTalonFX motor: motors) {
      setBrakeMode(motor, isEnabled);
    }
  }
  public static void setBrakeMode(CoreTalonFX motor, boolean isEnabled) {
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
    Pair<DeviceHealth, String> pair = getMotorHealthAndFaults(motor);
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
  public static Pair<DeviceHealth, String> getMotorHealthAndFaults(CoreTalonFX motor) {
    FaultCase[] faultCases = new FaultCase[] {
      new FaultCase(motor.getFault_Hardware().getValue(), DeviceHealth.INOPERABLE, " hardware_failure "),
      new FaultCase(motor.getFault_DeviceTemp().getValue(), DeviceHealth.INOPERABLE, " device_temperature_exceeded_limit "),
      new FaultCase(motor.getFault_ProcTemp().getValue(), DeviceHealth.INOPERABLE, " processor_temperature_exceeded_limit "),
      new FaultCase(motor.getFault_Undervoltage().getValue(), DeviceHealth.FAULT, " device_under_6.5v "),
      new FaultCase(motor.getFault_OverSupplyV().getValue(), DeviceHealth.FAULT, " supply_voltage_above_rated_max "),
      new FaultCase(motor.getFault_UnstableSupplyV().getValue(), DeviceHealth.FAULT, " unstable_supply_voltage "),
      new FaultCase(motor.getFault_BootDuringEnable().getValue(), DeviceHealth.FAULT, " device_boot_or_reset_while_robot_enabled "),
      new FaultCase(motor.getFault_MissingRemoteSensor().getValue(), DeviceHealth.FAULT, " remote_sensor_not_detected "),
      new FaultCase(motor.getFault_FusedSensorOutOfSync().getValue(), DeviceHealth.FAULT, " fused_sensor_out_of_sync "),
      new FaultCase(motor.getFault_UsingFusedCANcoderWhileUnlicensed().getValue(), DeviceHealth.FAULT, " using_fused_CANcoder_feature_while_unlicensed "),
      new FaultCase(motor.getFault_UnlicensedFeatureInUse().getValue(), DeviceHealth.FAULT, " unlicensed_feature_in_use "),
    };

    return getDeviceHealthAndFaults(faultCases);
  }

   /**
   * Get CANCoder faults and device health.
   * 
   * @param encoderFaults CANCoder faults.
   * @return CANCoder device health and error type (if any).
   */
  public static Pair<DeviceHealth, String> getCANcoderHealthAndFaults(CANcoder canCoder) {

    FaultCase[] faultCases = new FaultCase[] {
      new FaultCase(canCoder.getFault_Hardware().getValue(), DeviceHealth.INOPERABLE, " hardware_failure "),
      new FaultCase(canCoder.getFault_BadMagnet().getValue(), DeviceHealth.INOPERABLE, " magnet_too_weak "),
      new FaultCase(canCoder.getFault_Undervoltage().getValue(), DeviceHealth.FAULT, " device_under_6.5v "),
      new FaultCase(canCoder.getFault_BootDuringEnable().getValue(), DeviceHealth.FAULT, " device_boot_or_reset_while_robot_enabled "),
      new FaultCase(canCoder.getFault_UnlicensedFeatureInUse().getValue(), DeviceHealth.FAULT, " unlicensed_feature_in_use "),
    };

    return getDeviceHealthAndFaults(faultCases);
  }

  /**
   * @param canCoder
   * @return Pair<DeviceHealth, String>
   */
  public static Pair<DeviceHealth, String> checkCANcoderFaultsAndConnection(CANcoder canCoder) {
    Pair<DeviceHealth, String> pair = getCANcoderHealthAndFaults(canCoder);
    String retStr = pair.getSecond();
    DeviceHealth retHealth = pair.getFirst();
    if (canCoder.getVersion().getValue() == 0) {
      retStr += " device_disconnected ";
      retHealth = DeviceHealth.INOPERABLE;
    }
    return new Pair<DeviceHealth, String>(retHealth, retStr);
  }

  private static Pair<DeviceHealth, String> getDeviceHealthAndFaults(FaultCase... faultCases) {
    StringBuilder work = new StringBuilder();
    DeviceHealth health = DeviceHealth.NOMINAL;
    for (FaultCase faultCase: faultCases) {
      if (faultCase.flag) {
        work.append(faultCase.messsage);
        if (health.ordinal() < faultCase.effect.ordinal()) {
          health = faultCase.effect;
        }
      }
    }
    return new Pair<DeviceHealth,String>(health, work.toString());
  }


  private static class FaultCase {
    public boolean flag;
    public DeviceHealth effect;
    public String messsage;
    public FaultCase(boolean flag, DeviceHealth effect, String messsage) {
      this.flag = flag;
      this.effect = effect;
      this.messsage = messsage;
    }
  }

  
}
