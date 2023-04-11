package frc.robot.swerve;

import com.ctre.phoenixpro.configs.Slot0Configs;

public class SwerveModuleConstants {
    public int DriveMotorId = 0;
    public int SteerMotorId = 0;
    public int CANcoderId = 0;
    public double CANcoderOffset = 0;
    public double DriveMotorGearRatio = 0;
    public double SteerMotorGearRatio = 0;
    public double WheelRadius = 0;
    public double LocationX = 0;
    public double LocationY = 0;
    public Slot0Configs DriveMotorGains = new Slot0Configs();
    public Slot0Configs SteerMotorGains = new Slot0Configs();
    public double SlipCurrent = 400;
    public boolean SteerMotorReversed = false;
    public SwerveModuleConstants withDriveMotorId(int id) {
        DriveMotorId = id;
        return this;
    }
    public SwerveModuleConstants withSteerMotorId(int id) {
        SteerMotorId = id;
        return this;
    }
    public SwerveModuleConstants withCANcoderId(int id) {
        CANcoderId = id;
        return this;
    }
    public SwerveModuleConstants withCANcoderOffset(double offset) {
        CANcoderOffset = offset;
        return this;
    }
    public SwerveModuleConstants withDriveMotorGearRatio(double ratio) {
        DriveMotorGearRatio = ratio;
        return this;
    }
    public SwerveModuleConstants withSteerMotorGearRatio(double ratio) {
        SteerMotorGearRatio = ratio;
        return this;
    }
    public SwerveModuleConstants withWheelRadius(double radius) {
        WheelRadius = radius;
        return this;
    }
    public SwerveModuleConstants withLocationX(double x) {
        LocationX = x;
        return this;
    }
    public SwerveModuleConstants withLocationY(double y) {
        LocationY = y;
        return this;
    }
    public SwerveModuleConstants withDriveMotorGains(Slot0Configs gains) {
        DriveMotorGains = gains;
        return this;
    }
    public SwerveModuleConstants withSteerMotorGains(Slot0Configs gains) {
        SteerMotorGains = gains;
        return this;
    }
    public SwerveModuleConstants withSlipCurrent(double current) {
        SlipCurrent = current;
        return this;
    }
    public SwerveModuleConstants withSteerMotorReversed(boolean reversed) {
        SteerMotorReversed = reversed;
        return this;
    }

    public SwerveModuleConstants fromModuleConfiguration(ModuleConfiguration config) {
        return new SwerveModuleConstants()
            .withDriveMotorGearRatio(config.getDriveReduction())
            .withSteerMotorGearRatio(config.getSteerReduction())
            .withWheelRadius(config.getWheelDiameter() / 2)
            .withSteerMotorReversed(config.isSteerInverted());
    }
}
