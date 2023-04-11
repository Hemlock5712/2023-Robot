package frc.robot.swerve;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ProSwerveModule {
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder cancoder;

    private StatusSignalValue<Double> drivePosition;
    private StatusSignalValue<Double> driveVelocity;
    private StatusSignalValue<Double> steerPosition;
    private StatusSignalValue<Double> steerVelocity;
    private BaseStatusSignalValue[] signals;
    private double driveRotationsPerMeter = 0;

    private PositionVoltage angleSetter = new PositionVoltage(0);
    private VelocityTorqueCurrentFOC velocitySetter = new VelocityTorqueCurrentFOC(0);

    private SwerveModulePosition internalState = new SwerveModulePosition();

    public ProSwerveModule(SwerveModuleConstants constants) {
      this(constants, "rio");
    }

    public ProSwerveModule(SwerveModuleConstants constants, String canbusName) {
        this.driveMotor = new TalonFX(constants.DriveMotorId, canbusName);
        this.steerMotor = new TalonFX(constants.SteerMotorId, canbusName);
        this.cancoder = new CANcoder(constants.CANcoderId, canbusName);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.Slot0 = constants.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = constants.SlipCurrent;
        driveMotor.getConfigurator().apply(talonConfigs);

        talonConfigs.Slot0 = constants.SteerMotorGains;
        talonConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        talonConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        talonConfigs.MotorOutput.Inverted = constants.SteerMotorReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        steerMotor.getConfigurator().apply(talonConfigs);

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
        cancoder.getConfigurator().apply(cancoderConfigs);

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerPosition = steerMotor.getPosition();
        steerVelocity = steerMotor.getVelocity();

        signals = new BaseStatusSignalValue[] {drivePosition, driveVelocity, steerPosition, steerVelocity};

        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = constants.WheelRadius * 2 * Math.PI;
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    }

    public SwerveModulePosition getPosition() {
        drivePosition.refresh();
        driveVelocity.refresh();
        steerPosition.refresh();
        steerVelocity.refresh();

        // Compensate for latency
        double driveRotation = drivePosition.getValue() + (driveVelocity.getValue() * drivePosition.getTimestamp().getLatency());
        double steerRotation = steerPosition.getValue() + (steerVelocity.getValue() * steerPosition.getTimestamp().getLatency());

        internalState.distanceMeters = driveRotation / driveRotationsPerMeter;
        internalState.angle = Rotation2d.fromRotations(steerRotation);

        return internalState;
    }

    public void apply(SwerveModuleState state) {
        SwerveModuleState optimizedModuleState = SwerveModuleState.optimize(state, internalState.angle);

        double angleToSetDeg = optimizedModuleState.angle.getRotations();
        steerMotor.setControl(angleSetter.withPosition(angleToSetDeg));

        double velocityToSet = optimizedModuleState.speedMetersPerSecond * driveRotationsPerMeter;
        driveMotor.setControl(velocitySetter.withVelocity(velocityToSet));
    }

    BaseStatusSignalValue[] getSignals() {
        return signals;
    }
    
    public void setNeutralMode(NeutralModeValue neutralMode) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = neutralMode;
      driveMotor.getConfigurator().apply(config);
    }

    public Rotation2d getSteerAngle() {
      return internalState.angle;
    }

    public double getDriveVelocity() {
      return driveVelocity.getValue();
    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), getSteerAngle());
    }
}
