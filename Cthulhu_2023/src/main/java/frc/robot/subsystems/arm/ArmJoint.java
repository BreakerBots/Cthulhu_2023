package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmJoint extends TrapezoidProfileSubsystem {
  private WPI_TalonFX motor;
  private WPI_CANCoder encoder;
  private ArmFeedforward ff;
  private Supplier<Rotation2d> angleOffsetSupplier;

  /** Create a new ArmSubsystem. */
  public ArmJoint(Supplier<Rotation2d> angleOffsetSupplier, ArmJointConfig config) {
    super(
        config.constraints,
        Rotation2d.fromDegrees(config.encoder.getAbsolutePosition()).plus(angleOffsetSupplier.get()).getRadians());
    ff = new ArmFeedforward(config.kS, config.kG, config.kV, config.kA);
    motor = config.motors[0];
    encoder = config.encoder;
    this.angleOffsetSupplier = angleOffsetSupplier;
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.remoteFilter0.remoteSensorDeviceID = config.encoder.getDeviceID();
    motorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    motorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    motorConfig.slot0.kP = config.kP;
    motorConfig.slot0.kI = config.kI;
    motorConfig.slot0.kD = config.kD;
    motorConfig.slot0.closedLoopPeakOutput = 1.0;
    motorConfig.peakOutputForward = 1.0;
    motorConfig.peakOutputReverse = -1.0;
    motorConfig.voltageCompSaturation = 12.0;
    motorConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
    BreakerCTREUtil.checkError(motor.configAllSettings(motorConfig),
        " Failed to arm joint motor ");
    motor.selectProfileSlot(0, 0);
    if (config.motors.length > 0) {
      for (int i = 1; i < config.motors.length; i++) {
        config.motors[i].follow(motor);
      }
    }
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    motor.set(TalonFXControlMode.Position,
        radiansToCANCoderNativeUnits(new Rotation2d(setpoint.position).minus(getJointAngle()).getRadians()),
        DemandType.ArbitraryFeedForward, feedforward / RobotController.getBatteryVoltage());
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }

  public Rotation2d getJointAngle() {
    return Rotation2d.fromDegrees(encoder.getAbsolutePosition()).plus(angleOffsetSupplier.get());
  }

  /** rad/sec */
  public double getJointVel() {
    return Math.toDegrees(encoder.getVelocity()) * 10;
  }

  private double radiansToCANCoderNativeUnits(double angleRad) {
    return (angleRad / (2 * Math.PI)) * 4096.0;
  }

  public static class ArmJointConfig {
    public final double kP, kI, kD, kS, kG, kV, kA;
    public final TrapezoidProfile.Constraints constraints;
    public final WPI_TalonFX[] motors;
    public final WPI_CANCoder encoder;

    public ArmJointConfig(WPI_CANCoder encoder, double encoderOffsetDegrees, boolean invertEncoder,
        TrapezoidProfile.Constraints constraints, double kP, double kI, double kD, double kS, double kG, double kV,
        double kA, WPI_TalonFX... motors) {
      BreakerCANCoderFactory.configExistingCANCoder(encoder, SensorInitializationStrategy.BootToAbsolutePosition,
          AbsoluteSensorRange.Signed_PlusMinus180, encoderOffsetDegrees, invertEncoder);
      this.motors = motors;
      this.encoder = encoder;
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kS = kS;
      this.kG = kG;
      this.kV = kV;
      this.kA = kA;
      this.constraints = constraints;
    }
  }
}