// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;

/** Add your docs here. */
public class BreakerSwerveMovementPreferences {
    protected final SwerveMovementRefrenceFrame swerveMovementRefrenceFrame;
    protected final SlowModeValue slowModeValue;
    protected final boolean headingCorrectionEnabled;
    public static final BreakerSwerveMovementPreferences DEFAULT_ROBOT_RELATIVE_PREFERENCES = new BreakerSwerveMovementPreferences().withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame.ROBOT_RELATIVE);
    public static final BreakerSwerveMovementPreferences DEFAULT_FIELD_RELATIVE_PREFERENCES = new BreakerSwerveMovementPreferences();

    /** Uses the drivetrain as odometry provider and uses a field relative movement angle offset. */
    public BreakerSwerveMovementPreferences() {
        this(SwerveMovementRefrenceFrame.ROBOT_RELATIVE, SlowModeValue.DEFAULT, false);
    }

    public BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue) {
        this(movementRefrenceFrame, slowModeValue, false);
    } 

    protected BreakerSwerveMovementPreferences(SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, boolean headingCorrectionEnabled) {
      this.slowModeValue = slowModeValue;
      this.swerveMovementRefrenceFrame = movementRefrenceFrame;
      this.headingCorrectionEnabled = headingCorrectionEnabled;
    } 

    public BreakerSwerveMovementPreferences withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame) {
        return new BreakerSwerveMovementPreferences(swerveMovementRefrenceFrame, this.slowModeValue, this.headingCorrectionEnabled);
    }

    /** Sets whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
     * @param slowModeValue Whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
     * @return this.
     */
    public BreakerSwerveMovementPreferences withSlowModeValue(SlowModeValue slowModeValue) {
        return new BreakerSwerveMovementPreferences(this.swerveMovementRefrenceFrame, slowModeValue, this.headingCorrectionEnabled);
    }

    public SlowModeValue getSlowModeValue() {
        return slowModeValue;
    }

    public SwerveMovementRefrenceFrame getSwerveMovementRefrenceFrame() {
        return swerveMovementRefrenceFrame;
    }

    public boolean getHeadingCorrectionEnabled() {
        return headingCorrectionEnabled;
    }

    public enum SwerveMovementRefrenceFrame {
        FIELD_RELATIVE_WITH_OFFSET,
        FIELD_RELATIVE_WITHOUT_OFFSET,
        ROBOT_RELATIVE
    }
}
