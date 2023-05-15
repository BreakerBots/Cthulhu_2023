// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.smart;

import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.BreakerLib.devices.motors.smart.BreakerSmartMotorControler.BreakerSmartMotorControlRequest.BreakerSmartMotorControlRequestOutputMode;
import frc.robot.BreakerLib.devices.motors.smart.BreakerSmartMotorControler.BreakerSmartMotorControlRequest.BreakerSmartMotorControlRequestStatusCode;
import frc.robot.BreakerLib.devices.motors.smart.BreakerSmartMotorControler.BreakerSmartMotorControlRequest.BreakerSmartMotorControlRequestType;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestable;

/** Add your docs here. */
public interface BreakerSmartMotorControler extends MotorController, Sendable, AutoCloseable, BreakerSelfTestable {
    

    public abstract BreakerSmartMotorControlRequestStatusCode setControl(BreakerSmartMotorControlRequest request);


    public static class BreakerSmartMotorControlRequest {
        public enum BreakerSmartMotorControlRequestType {
            OUTPUT,
            POSITION,
            VELOCITY,
            SMART_MOTION
        }

        public enum BreakerSmartMotorControlRequestOutputMode {
            VOLTAGE,
            DUTY_CYCLE,
            CURRENT
        }

        public enum BreakerSmartMotorControlRequestStatusCode {
            OK,
            FALLBACK,
            FAIL
        }

        private BreakerSmartMotorControlRequestType requestType;
        private BreakerSmartMotorControlRequestOutputMode requestOutputMode;
        private boolean enableFOC, enableRequestFallback;
        private double requestValue, feedforwardValue;
        private int pidSlot;

        public BreakerSmartMotorControlRequest(BreakerSmartMotorControlRequestType requestType, BreakerSmartMotorControlRequestOutputMode requestOutputMode, boolean enableFOC, boolean enableRequestFallback, int pidSlot, double requestValue, double feedforwardValue) {
            this.enableFOC = enableFOC;
            this.enableRequestFallback = enableRequestFallback;
            this.feedforwardValue = feedforwardValue;
            this.requestOutputMode = requestOutputMode;
            this.requestType  = requestType;
            this.requestValue = requestValue;
            this.pidSlot = pidSlot;
        }

        public double getFeedforwardValue() {
            return feedforwardValue;
        }

        public BreakerSmartMotorControlRequestOutputMode getRequestOutputMode() {
            return requestOutputMode;
        }
        public BreakerSmartMotorControlRequestType getRequestType() {
            return requestType;
        }

        public double getRequestValue() {
            return requestValue;
        }

        public boolean getEnableFOC() {
            return enableFOC;
        }

        public boolean getEnableRequestFallback() {
            return enableRequestFallback;
        }

        public int getPidSlot() {
            return pidSlot;
        }

        public BreakerSmartMotorControlRequest withValue(double requestValue) {
            this.requestValue = requestValue;
            return this;
        }

        public BreakerSmartMotorControlRequest withFeedforward(double feedforwardValue) {
            this.feedforwardValue = feedforwardValue;
            return this;
        }

        public BreakerSmartMotorControlRequest withEnableFOC(boolean enableFOC) {
            this.enableFOC = enableFOC;
            return this;
        }

        public BreakerSmartMotorControlRequest withEnableRequestFallback(boolean enableRequestFallback) {
            this.enableRequestFallback = enableRequestFallback;
            return this;
        }

        public BreakerSmartMotorControlRequest withPidSlot(int pidSlot) {
            this.pidSlot = pidSlot;
            return this;
        }

        public BreakerSmartMotorControlRequest withRequestOutputMode(BreakerSmartMotorControlRequestOutputMode requestOutputMode) {
            this.requestOutputMode = requestOutputMode;
            return this;
        }

        public BreakerSmartMotorControlRequest withRequestType(BreakerSmartMotorControlRequestType requestType) {
            this.requestType = requestType;
            return this;
        }
    }
}
