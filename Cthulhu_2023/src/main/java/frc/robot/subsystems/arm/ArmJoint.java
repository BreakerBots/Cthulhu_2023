package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;
import io.github.oblarg.oblog.Loggable;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmJoint extends SubsystemBase implements Loggable {
  private WPI_TalonFX motor;
  private WPI_CANCoder encoder;
  private ArmFeedforward ff;
  private PIDController pid;
  private Supplier<Rotation2d> angleOffsetSupplier;
  private Supplier<BreakerVector2> vec2Supplier;
  private double armLengthMeters;
  public double maxAng, minAng;
  private boolean enabled = true;

  /** Create a new ArmSubsystem. */
  private Rotation2d target;
  public ArmJoint(Supplier<Rotation2d> angleOffsetSupplier, double armLengthMeters, ArmJointConfig config) {
    pid = new PIDController(config.kP, config.kI,config.kD);
    pid.setTolerance(2, Double.MAX_VALUE);
    ff = new ArmFeedforward(config.kS, config.kG, config.kV, config.kA);
    motor = config.motors[0];
    encoder = config.encoder;
    this.angleOffsetSupplier = angleOffsetSupplier;
    this.vec2Supplier = () -> {return new BreakerVector2();};
    this.armLengthMeters = armLengthMeters;
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.peakOutputForward = 0.35;
    motorConfig.peakOutputReverse = -0.35;
    motorConfig.voltageCompSaturation = 12.0;
    motorConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
    BreakerCTREUtil.checkError(motor.configAllSettings(motorConfig),
        " Failed to arm joint motor ");
        motor.setInverted(config.invertMotor);
    target = getJointAngle();
    motor.selectProfileSlot(0, 0);
    if (config.motors.length > 0) {
      for (int i = 1; i < config.motors.length; i++) {
        config.motors[i].follow(motor);
      }
    }
  }

  public void setAttacedJointVecSupplier(Supplier<BreakerVector2>  vec2Supplier) {
    this.vec2Supplier = vec2Supplier;
  }

  public BreakerVector2 getJointVector() {
    return BreakerVector2.fromForceAndRotation(getJointAngle(), armLengthMeters);
  }

  public Rotation2d getJointAngle() {
    return Rotation2d.fromDegrees(encoder.getAbsolutePosition()).plus(angleOffsetSupplier.get());
  }

  /** rad/sec */
  public double getJointVel() {
    return Math.toDegrees(encoder.getVelocity()) * 10;
  }

  public double getRawMotorOut() {
    return motor.getMotorOutputPercent();
  }

  private double radiansToCANCoderNativeUnits(double angleRad) {
    return (angleRad / (2 * Math.PI)) * 4096.0;
  }

  public boolean getUpLimit() {
    return getJointAngle().getDegrees() > maxAng;
  }

  public boolean getDownLimit() {
    return getJointAngle().getDegrees() < minAng;
  }

  public void setTarget(Rotation2d target) {
    this.target = target;
  }

  public void setEnabled(boolean enabled) {
      this.enabled = enabled;
  }

  public boolean isEnabled() {
      return enabled;
  }

  public void periodic() {
    SmartDashboard.putNumber("MOTOR OUT", motor.get());
    SmartDashboard.putData(pid);
    pid.calculate(getJointAngle().getDegrees(), target.getDegrees());
    if (!pid.atSetpoint()) {
      motor.set(-0.12);
    } else {
      motor.set(-0.05);
    }
  }

  public static class ArmJointConfig {
    public final double kP, kI, kD, kS, kG, kV, kA;
    public final TrapezoidProfile.Constraints constraints;
    public final WPI_TalonFX[] motors;
    public final WPI_CANCoder encoder;
    public final boolean invertMotor;

    public ArmJointConfig(WPI_CANCoder encoder, double encoderOffsetDegrees, boolean invertEncoder, boolean invertMotor,
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
      this.invertMotor = invertMotor;
    }
  }
}