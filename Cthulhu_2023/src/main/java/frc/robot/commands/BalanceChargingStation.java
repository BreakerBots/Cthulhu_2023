// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Odometer;

public class BalanceChargingStation extends CommandBase {
  /** Creates a new BalanceChargingStation. */
  private BreakerPigeon2 imu;
  private PIDController balencePID;
  //private Odometer odometer;
  private Drive drivetrain;
  
  public BalanceChargingStation(Drive drivetrain, BreakerPigeon2 imu) { /* Odometer odometer */
    // Use addRequirements() here to declare subsystem dependencies.
    this.imu = imu;
    //this.odometer = odometer;
    this.drivetrain = drivetrain;
    balencePID = new PIDController(0.001, 0, 0);
    balencePID.setTolerance(2.0, 0.05);
    addRequirements(drivetrain);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double corSpeed = balencePID.calculate(imu.getPitchDegrees(), 0.0);
    // if (DriverStation.getAlliance() == Alliance.Red) {
    //   corSpeed *= -1.0;
    // }
    // if (outOfWorkingBounds()) {
    //   corSpeed = outOfWorkingBoundsSpeedClamp(corSpeed);
    // }
    drivetrain.moveRelativeToField(corSpeed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // private boolean outOfWorkingBounds() {
  //   if (DriverStation.getAlliance() == Alliance.Blue) {
  //     return !Constants.Field.BLUE_CHARGING_STATION_WORKING_BOUNDS.contains(odometer.getRobotHitbox());
  //   } else {
  //     return !Constants.Field.RED_CHARGING_STATION_WORKING_BOUNDS.contains(odometer.getRobotHitbox());
  //   }
  // }

  // private double outOfWorkingBoundsSpeedClamp(double speed) {
  //   double fwdBound = 0.0;
  //   double revBound = 0.0;
  //   double fwdRobotBound = 0.0;
  //   double revRobotBound = 0.0;
  //   if (DriverStation.getAlliance() == Alliance.Blue) {
  //     fwdBound = Constants.Field.BLUE_CHARGING_STATION_WORKING_BOUNDS.getMaxY();
  //     revBound = Constants.Field.BLUE_CHARGING_STATION_WORKING_BOUNDS.getMinY();
  //     fwdRobotBound = odometer.getRobotHitbox().getMaxY();
  //     revRobotBound = odometer.getRobotHitbox().getMinY();
  //   } else {
  //     fwdBound = Constants.Field.BLUE_CHARGING_STATION_WORKING_BOUNDS.getMinY();
  //     revBound = Constants.Field.BLUE_CHARGING_STATION_WORKING_BOUNDS.getMaxY();
  //     fwdRobotBound = odometer.getRobotHitbox().getMinY();
  //     revRobotBound = odometer.getRobotHitbox().getMaxY();
  //   } 
  //   if (fwdBound <= fwdRobotBound) {
  //     return MathUtil.clamp(speed, Double.MAX_VALUE, 0.0);
  //   } else if (revBound >= revRobotBound) {
  //     return MathUtil.clamp(speed, 0.0, Double.MAX_VALUE);
  //   } else {
  //     return speed;
  //   }
  // }

  // private boolean outOfBounds() {
  //   if (DriverStation.getAlliance() == Alliance.Blue) {
  //     return !Constants.Field.BLUE_CHARGING_STATION.contains(odometer.getRobotHitbox());
  //   } else {
  //     return !Constants.Field.RED_CHARGING_STATION.contains(odometer.getRobotHitbox());
  //   }
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return /*outOfBounds() ||*/ balencePID.atSetpoint();
  }
}
