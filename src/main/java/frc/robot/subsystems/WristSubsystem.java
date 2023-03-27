package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private CANSparkMax wristMotor = new CANSparkMax(Constants.WristConstants.MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private CANCoder wristEncoder = new CANCoder(Constants.WristConstants.ENCODER_ID);
    // These constants are lower than they should be to prevent the wrist from going
    // too far instantly
    private PIDController wristPID = new PIDController(.1, 0, 0);
    // These constants are calculated by Reca.lc, might need to be tuned slightly
    // private ArmFeedforward wristFeedforward = new ArmFeedforward(0, 2.62, 0.48,
    // 0.07);

    private double backupEncoderOffsetAngle = 0;
    private ArmFeedforward wristFeedforward = new ArmFeedforward(0, .2, .79, .04);

    private NetworkTableEntry wristTargetAngleEntry = NetworkTableInstance.getDefault().getTable("Wrist")
            .getEntry("targetAngle");
    private NetworkTableEntry wristCurrentAngleEntry = NetworkTableInstance.getDefault().getTable("Wrist")
            .getEntry("currentAngle");
    private NetworkTableEntry wristBackupAngleEntry = NetworkTableInstance.getDefault().getTable("Wrist")
            .getEntry("backupAngle");
    private NetworkTableEntry wristVoltageEntry = NetworkTableInstance.getDefault().getTable("Wrist").getEntry("voltage");
    private NetworkTableEntry wristCurrentEntry = NetworkTableInstance.getDefault().getTable("Wrist").getEntry("current");
    private NetworkTableEntry wristTemperatureEntry = NetworkTableInstance.getDefault().getTable("Wrist")
            .getEntry("temperature");
    private NetworkTableEntry atSetpointEntry = NetworkTableInstance.getDefault().getTable("Wrist")
            .getEntry("atSetpoint");

    private double setpoint = 0;
    private boolean isRunningBackup = false;
    private boolean wasRunningBackup = false;

    public WristSubsystem() {
        wristEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        wristEncoder.configMagnetOffset(-50);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setSmartCurrentLimit(30, 80);
        // Set up backup encoder to default position. This will only be used if the CANCoder has issues mid match
        backupEncoderOffsetAngle = wristEncoder.getAbsolutePosition();
        if (wristEncoder.getLastError() != ErrorCode.OK) {
            // Set the backup encoder to the starting position of the arm if there's a CANCoder issue on startup
            // Also will send an alert to the driver station about there being an issue
            System.err.println("Error reading value from wrist encoder. Defaulting to backup encoder with current angle of " + Constants.ArmSetpoints.STARTING_CONFIG.getWristAngle());
            backupEncoderOffsetAngle = Constants.ArmSetpoints.STARTING_CONFIG.getWristAngle();
        }
    }

    public void setTargetAngle(double angle) {
        this.setpoint = angle;
    }

    public double getTargetAngle() {
        return this.setpoint;
    }

    public double getAngle() {
        double angle = wristEncoder.getAbsolutePosition();
        if (isRunningBackup || wristEncoder.getLastError() != ErrorCode.OK) {
            // If there's an error reading from the CANCoder, use the backup encoder.
            isRunningBackup = true;
            angle = getBackupAngle();
        }
        return angle;
    }

    /**
     * Returns the angle that the wrist is at using the built in encoder on the NEO550
     * <p>
     * Should only be used if a catastrophic failure occurs with the CANCoder
     *
     * @return The angle of the wrist in degrees
     */
    private double getBackupAngle() {
        return wristMotor.getEncoder().getPosition() * 360
                / (Constants.WristConstants.TICKS_PER_REVOLUTION *
                Constants.WristConstants.GEAR_RATIO) + backupEncoderOffsetAngle;
    }

    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    public boolean atTarget() {
        return Math.abs(getAngle() - setpoint) < 4;
    }

    public void run() {
        double feedforward = wristFeedforward.calculate(Math.toRadians(setpoint), 0);
        // + feedforward
        // double output = feedforward;
        double pid = wristPID.calculate(getAngle(), setpoint);
        SmartDashboard.putNumber("WristPID", pid);
        SmartDashboard.putNumber("WristFF", feedforward);
        double output = feedforward + pid;
        SmartDashboard.putNumber("WristOutput", output);
        // System.out.println(feedforward);
        // System.out.println("_________ " + output);

        setVoltage(output);
        wristCurrentAngleEntry.setDouble(getAngle());
        wristBackupAngleEntry.setDouble(getBackupAngle());
        wristTargetAngleEntry.setDouble(setpoint);
        wristVoltageEntry.setDouble(wristMotor.getAppliedOutput());
        wristCurrentEntry.setDouble(wristMotor.getOutputCurrent());
        atSetpointEntry.setBoolean(atTarget());
        if(isRunningBackup != wasRunningBackup) {
            System.err.println("WristSubsystem is running backup encoder. This needs to be looked at ASAP");
            wasRunningBackup = true;
        }
    }

    @Override
    public void periodic() {
        run();
        // double feedforward = wristFeedforward.calculate(setpoint, 0);
        // double output = wristPID.calculate(getAngle(), setpoint) + feedforward;
        // // setVoltage(output);
//         wristCurrentAngleEntry.setDouble(getAngle());
        // wristTargetAngleEntry.setDouble(setpoint);
        // wristVoltageEntry.setDouble(wristMotor.getAppliedOutput());
        // wristCurrentEntry.setDouble(wristMotor.getOutputCurrent());
        wristTemperatureEntry.setDouble(wristMotor.getMotorTemperature());


    }
}
