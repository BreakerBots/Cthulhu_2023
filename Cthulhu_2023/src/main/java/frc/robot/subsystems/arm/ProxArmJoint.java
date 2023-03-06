package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.util.math.BreakerMath;

public class ProxArmJoint extends ArmJoint {


    public ProxArmJoint(Supplier<Rotation2d> angleOffsetSupplier, double armLengthMeters, ArmJointConfig config) {
      super(angleOffsetSupplier, armLengthMeters, config);
    }

    @Override
    public void periodic() {
    SmartDashboard.putData(pid);
    pid.calculate(getJointAngle().getDegrees(), target.getDegrees());
    double err = pid.getPositionError();
    if (isEnabled()) {
      if (!pid.atSetpoint()) {
        if (BreakerMath.epsilonEquals(encoder.getVelocity(), 0, 2.0)){
          motor.set(Math.signum(err) * -1);
        } else {
          motor.set(Math.signum(err) * pid.getP());
        }
      } else {
        motor.set(-0.05);
      }
    } else {
      motor.set(0);
    }
    }
}