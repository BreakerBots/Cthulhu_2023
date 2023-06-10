// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;


// import static frc.robot.Constants.ArmConstants.*;
// import static frc.robot.Constants.MiscConstants.CANIVORE_1;

// import java.util.Map;

// import com.ctre.phoenix6.configs.CustomParamsConfigs;
// import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
// import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.ReverseLimitSourceValue;

// /** Add your docs here. */
// public class Phoenix6FalconArm extends ProfiledPIDSubsystem {

//     public static final double kS = 0.0;
//     public static final double kG = 0.675;
//     public static final double kV = 0;
//     public static final double kA = 0.0;
//     public static final double kP = 0;
//     public static final double kI = 0.0;
//     public static final double kD = 0.0;

//     public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2, 0.5);

//     private TalonFX motor0 = new TalonFX(MAIN_MOTOR_ID, CANIVORE_1);
//     private TalonFX motor1 = new TalonFX(SUB_MOTOR_ID, CANIVORE_1);
//     private CANcoder encoder = new CANcoder(ARM_CANCODER_ID);
//     private ArmFeedforward armFF = new ArmFeedforward(kS, kG, kV, kA);
//     private static ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, constraints);

//     public Phoenix6FalconArm() {

//         super(pid);
//         enable();
//         setGoal(getMeasurement());
//         motor1.setControl(new Follower(motor0.getDeviceID(), false));

//         TalonFXConfigurator m0Config = motor0.getConfigurator();
//         TalonFXConfigurator m1Config = motor1.getConfigurator();

//         MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
//         outputConfigs.NeutralMode = NeutralModeValue.Coast;

//         HardwareLimitSwitchConfigs hardLimitConfig = new HardwareLimitSwitchConfigs();
//         hardLimitConfig.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
//         hardLimitConfig.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
//         hardLimitConfig.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;

//         m0Config.apply(outputConfigs);
//         m1Config.apply(outputConfigs);
//         motor0.setInverted(true);

//         motor0.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
//         motor0.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
       
//         encoder.configSensorDirection(false);
//         encoder.configMagnetOffset(ARM_CANCODER_OFFSET);
//         encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
//         encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

//     }

//     @Override
//     protected void useOutput(double output, State setpoint) {
//         double ff = armFF.calculate(setpoint.position, setpoint.velocity);
//         SmartDashboard.putNumber("Motor In", ff + output);
//         motor0.setVoltage(ff + output);
//     }

//     @Override
//     protected double getMeasurement() {
//         return getPosRad();
//     }

//     public double getPosDeg() {
//         return encoder.getAbsolutePosition()
//                 + (encoder.getAbsolutePosition() <= -90 && encoder.getAbsolutePosition() >= -180 ? 360 : 0);
//     }

//     public double getPosRad() {
//         return Units.degreesToRadians(getPosDeg());
//     }

//     public void periodic() {
//         SmartDashboard.putNumber("Current Angle", getPosDeg());
//         SmartDashboard.putNumber("Target Angle", m_controller.getGoal().position);
//         SmartDashboard.putNumber("Motor Out", motor0.getMotorOutputVoltage());

//         if (m_enabled) {
//             useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
//         }
//     }

// }
