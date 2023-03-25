// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.functions.BreakerBezierCurve;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.MiscConstants.*;

public class SebArm extends SubsystemBase {
  public enum State {
    STOW_CUBE(Rotation2d.fromDegrees(210)),
    STOW_CONE(Rotation2d.fromDegrees(190)),
    PLACE_LOW(Rotation2d.fromDegrees(-26)),
    PICKUP_HIGH(Rotation2d.fromDegrees(175)),
    PLACE_CUBE_MID(Rotation2d.fromDegrees(25)),
    PLACE_CONE_MID(Rotation2d.fromDegrees(57)),
    PICKUP_LOW_CUBE(Rotation2d.fromDegrees(-48)),

    PICKUP_LOW_CONE_PREP(Rotation2d.fromDegrees(-26)),
    PICKUP_LOW_CONE(Rotation2d.fromDegrees(-45)),
    UNKNOWN(new Rotation2d());

    public final Rotation2d rot;

    private State(Rotation2d rot) {
      this.rot = rot;
    }
  }

  /** Creates a new NewArmTest. */

  // Motor reverse moves arm to intake position. Motor forward moves arm to stow
  // position.

  private WPI_TalonFX motor0, motor1;
  private Rotation2d desiredRot;
  private WPI_CANCoder canCoder;
  private PIDController pid;
  private ProfiledPIDController profPID;
  private BreakerBezierCurve curve;

  private boolean isConePreparing = true;

  private BreakerXboxController controller;



  /**
   * Arm class for the new arm.
   * 
   * @param controller
   */
  public SebArm(BreakerXboxController controller) {
    this.controller = controller;
    this.curve = new BreakerBezierCurve(new Translation2d(0.627, 0.038), new Translation2d(0.914, 0.632));

    motor0 = new WPI_TalonFX(MAIN_MOTOR_ID, CANIVORE_1);
    motor1 = new WPI_TalonFX(SUB_MOTOR_ID, CANIVORE_1);
    motor1.follow(motor0);

    motor0.setNeutralMode(NeutralMode.Brake);
    motor1.setNeutralMode(NeutralMode.Brake);
    // motor0.setNeutralMode(NeutralMode.Coast);
    // motor1.setNeutralMode(NeutralMode.Coast);
    motor1.setStatusFramePeriod(0, 0, 0);
    motor0.setInverted(TalonFXInvertType.CounterClockwise);
    motor1.setInverted(TalonFXInvertType.FollowMaster);

    motor0.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    motor0.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    canCoder = new WPI_CANCoder(ARM_CANCODER_ID);
    canCoder.configSensorDirection(false);
    canCoder.configMagnetOffset(ARM_CANCODER_OFFSET);
    canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    canCoder.setPosition(canCoder.getAbsolutePosition() +  (canCoder.getAbsolutePosition() <= -90 && canCoder.getAbsolutePosition() >= -180 ? 360 : 0));
    
    pid = new PIDController(0.02, 0, 0); // TODO: Set actual PID values for this constructor.

    // profPID = new ProfiledPIDController(0.02, 0.0, 0.0, new TrapezoidProfile.Constraints(100.0,20.0));

    //profPID.reset(canCoder.getPosition());
    desiredRot = Rotation2d.fromDegrees(canCoder.getPosition());
  }

  // public Rotation2d getAngle() {
  //   double ang = canCoder.getAbsolutePosition() + (canCoder.getAbsolutePosition() >= -90 && canCoder.getAbsolutePosition() <= -180 ? 360 : 0);
  //   return Rotation2d.fromDegrees(ang);
  // }

  public void setTarget(Rotation2d target) {
    desiredRot = target;
    //profPID.reset(canCoder.getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", canCoder.getPosition());
   //double ctrlInput = controller.getRightTrigger().get() - controller.getLeftTrigger().get();
    // motor0.set(-curve.getSignRelativeValueAtX(ctrlInput));
    double pos = canCoder.getPosition();
    //double ctrlInput = profPID.calculate(pos, desiredRot.getDegrees());
    double ctrlInput = pid.calculate(pos, desiredRot.getDegrees());
    if (!isAtTarget()) {
      motor0.set(-MathUtil.clamp(ctrlInput, -1.0, 1.0));
    } else {
       motor0.set(0);
     }
    // motor0.set(-MathUtil.clamp(ctrlInput, -1.0, 1.0));
    SmartDashboard.putNumber("Arm Tgt", desiredRot.getDegrees());
    SmartDashboard.putNumber("Arm motor", motor0.get());
    
  }

  public void setArmState(State armState) {
    setTarget(armState.rot);
  }

  public Rotation2d getTarget() {
    return desiredRot;
  }

  public boolean isAtTarget() {
    return BreakerMath.epsilonEquals(canCoder.getPosition(), desiredRot.getDegrees(), 2.0);
  }

  public void pickupLow() {
    if (RobotContainer.isInCubeMode()) {
      isConePreparing = true;
      setArmState(State.PICKUP_LOW_CUBE);
    } else {
      if (isConePreparing) {
        setArmState(State.PICKUP_LOW_CONE_PREP);
      } else {
        setArmState(State.PICKUP_LOW_CONE);
      }
      isConePreparing = !isConePreparing;
    }
  }

  public InstantCommand pickupLowCommand() {
    return new InstantCommand(this::pickupLow);
  }

  public void placeMid() {
    if (RobotContainer.isInCubeMode()) {
      setArmState(State.PLACE_CUBE_MID);
    } else {
      setArmState(State.PLACE_CONE_MID);
    }
  }

  public InstantCommand placeMidCommand() {
    return new InstantCommand(this::placeMid);
  }

  public void stow() {
    if (RobotContainer.isInCubeMode()) {
      setArmState(State.STOW_CUBE);
    } else {
      setArmState(State.STOW_CONE);
    }
  }

  public InstantCommand stowCommand() {
    return new InstantCommand(this::stow);
  }

  public void placeLow() {
    setArmState(State.PLACE_LOW);
  }

  public InstantCommand placeLowCommand() {
    return new InstantCommand(this::placeLow);
  }

  public InstantCommand setStateCommand(State target) {
    return new InstantCommand(() -> setArmState(target));
  }
}
