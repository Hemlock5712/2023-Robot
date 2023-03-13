package frc.robot.util;

public final class ArmSetpoint {
    private double angle;
    private double length;
    private double wristAngle;

    public ArmSetpoint(double angle, double length, double wristAngle) {
        this.angle = angle;
        this.length = length;
        this.wristAngle = wristAngle;
    }

    public double getAngle() {
        return angle;
    }

    public double getLength() {
        return length;
    }

    public double getWristAngle() {
        return wristAngle;
    }
}
