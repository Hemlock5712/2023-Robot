package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.util.subsystems.ElevatorSubsystemBase;

public class ElevatorSubsystem extends ElevatorSubsystemBase {

    CANCoder angleEncoder = new CANCoder(Constants.ArmConstants.ARM_ANGLE_ENCODER_ID);
    TalonFX elevatorMotorFront = new TalonFX(Constants.ArmConstants.ELEVATOR_FRONT_FALCON_ID);
    TalonFX elevatorMotorBack = new TalonFX(Constants.ArmConstants.ELEVATOR_BACK_FALCON_ID);

    NetworkTableEntry angleEntry = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem").getEntry("angle");
    NetworkTableEntry lengthEntry = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem").getEntry("length");
    NetworkTableEntry motorVoltageEntry = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem").getEntry("currentVoltage");

    /**
     * Creates a new ElevatorSubsystem.
     *
     * This is a test using state space control to control the elevator,
     * but can easily be replaced with a PID controller
     */
    public ElevatorSubsystem() {
        super(DCMotor.getFalcon500(2), Constants.ArmConstants.DRUM_RADIUS, Constants.ArmConstants.ARM_MASS, Constants.ArmConstants.MAX_SPEED, Constants.ArmConstants.MAX_ACCELERATION, Constants.ArmConstants.ELEVATOR_GEARING);
        elevatorMotorBack.follow(elevatorMotorFront);
    }

    /**
     * Gets the current height of the point where the elevator mounts with the arm
     * @return Current height, based on absolute CANCoder readings
     */
    @Override
    public double getHeight() {
        // Calculate the Y position of where the mounting point is, based off the current angle
        double y = Math.sin(getAngle()) * Constants.ArmConstants.MOUNT_POINT_DISTANCE_ON_ARM;
        // Return the y component of the arm position, with the pivot point added.
        // This should give the absolute position of the arm mounting change
        return y + Constants.ArmConstants.PIVOT_POINT_HEIGHT;
    }

    /**
     * Sets the target height that the elevator should attempt to go to
     *
     * The State Space model should automatically handle the movement to this position
     */
    @Override
    public void setTargetHeight(double height) {
        this.goal = new TrapezoidProfile.State(height, 0.0);
    }

    /**
     * Sets the voltage of the elevator motor
     *
     * Technically this is a percent output, but it is scaled to 12V since Falcons don't support voltage control
     *
     * Might be worth pulling the current voltage from the PDH and scaling it to that for consistency
     * @param voltage Voltage to set the motor to
     */
    @Override
    public void setMotorVoltage(double voltage) {
        elevatorMotorFront.set(ControlMode.PercentOutput, voltage / 12);
    }

    @Override
    public boolean atTarget() {
        return Math.abs(getHeight() - goal.position) < Constants.ArmConstants.AT_TARGET_TOLERANCE;
    }

    /**
     * Gets the current angle of the arm, with 0 being the horizontal position parallel to the ground,
     * and positive angles being upwards
     *
     * @return Current angle, based on absolute CANCoder readings
     */
    public double getAngle() {
        return angleEncoder.getAbsolutePosition() + Constants.ArmConstants.ARM_ANGLE_ABSOLUTE_OFFSET;
    }

    /**
     * Update NetworkTable entries for debugging purposes
     */
    @Override
    public void periodic() {
        super.periodic();
        angleEntry.setDouble(getAngle());
        lengthEntry.setDouble(getHeight());
        motorVoltageEntry.setDouble(elevatorMotorFront.getMotorOutputVoltage());
    }
}
