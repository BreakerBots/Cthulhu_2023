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
public class DistalArmJoint extends SubsystemBase implements Loggable {
  public WPI_TalonFX motor;
  protected WPI_CANCoder encoder;
  protected ArmFeedforward ff;
  protected PIDController pid;
  protected Supplier<Rotation2d> angleOffsetSupplier;
  protected Supplier<BreakerVector2> vec2Supplier;
  protected double armLengthMeters;
  public double maxAng, minAng;
  protected boolean enabled = true;

  /** Create a new ArmSubsystem. */
  protected Rotation2d target;
  public DistalArmJoint(Supplier<Rotation2d> angleOffsetSupplier, double armLengthMeters, WPI_CANCoder encoder, double encoderOffsetDegrees, boolean invertEncoder, boolean invertMotor,
  TrapezoidProfile.Constraints constraints, double kP, double kI, double kD, double kS, double kG, double kV,
  double kA, WPI_TalonFX... motors) {
    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(2, Double.MAX_VALUE);
    ff = new ArmFeedforward(kS, kG, kV, kA);
    motor = motors[0];
    this.encoder = encoder;
    this.angleOffsetSupplier = angleOffsetSupplier;
    this.vec2Supplier = () -> {return new BreakerVector2();};
    this.armLengthMeters = armLengthMeters;
        motor.setInverted(invertMotor);
    target = getJointAngle();
    motor.selectProfileSlot(0, 0);
    if (motors.length > 0) {
      for (int i = 1; i < motors.length; i++) {
        motors[i].follow(motor);
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
    return getRawJointAngle().plus(angleOffsetSupplier.get());
  }

  /** rad/sec */
  public double getJointVel() {
    return Math.toDegrees(encoder.getVelocity()) * 10;
  }

  public Rotation2d getRawJointAngle() {
    return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
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

  @Override
  public void periodic() {
    SmartDashboard.putData(pid);
    pid.calculate(getJointAngle().getDegrees(), target.getDegrees());
    double err = pid.getPositionError();
    if (isEnabled()) {
      if (!pid.atSetpoint()) {
        motor.set(Math.signum(err) * (pid.getP() * (err > 35 ? 1.5 : 1.0)));
      } else {
        motor.set(-0.05);
      }
    } else {
      motor.set(0);
    }
  }
}