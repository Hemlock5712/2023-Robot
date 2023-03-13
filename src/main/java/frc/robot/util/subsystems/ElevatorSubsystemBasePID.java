package frc.robot.util.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ElevatorSubsystemBasePID extends SubsystemBase {
  protected double elevatorGearRatio;
  protected PIDController pidController;
  protected ElevatorFeedforward feedforward;
  protected boolean autoPosition = false;
  protected double setpoint = 0;
  protected boolean hasValidSetpoint = false;

  public ElevatorSubsystemBasePID(double kP, double kI, double kD, double kS, double kG, double kV, double kA,
      double elevatorGearing) {
    elevatorGearRatio = elevatorGearing;
    pidController = new PIDController(kP, kI, kD);
    feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  }

  public abstract double getHeight();

  public abstract void setTargetHeight(double height);

  public abstract void setMotorVoltage(double voltage);

  public abstract boolean atTarget();

  @Override
  public void periodic() {
    // System.out.println("AutoPosition:" + autoPosition);
    // System.out.println("HasValidSetpoint:" + hasValidSetpoint);
    if (autoPosition) {
      if (hasValidSetpoint) {
        pidController.setSetpoint(setpoint);
        double nextVoltage = pidController.calculate(getHeight()) + feedforward.calculate(getHeight());
        setMotorVoltage(nextVoltage);
      } else {
        pidController.setSetpoint(getHeight());
        double nextVoltage = pidController.calculate(getHeight()) + feedforward.calculate(getHeight());
        setMotorVoltage(nextVoltage);
      }
    }
  }
}
