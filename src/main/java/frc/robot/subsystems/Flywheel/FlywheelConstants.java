package frc.robot.subsystems.Flywheel;

public final class FlywheelConstants {
    public static final int kFlywheelMainCanId = 21;
    public static final int kFlywheelFollowerCanID = 22;
    public static final double kFlywheelMaxMPS = 25; // Example max RPM for the flywheel motor
    // public static final double kFlywheelTargetRPM = 4000.0; // Target RPM for shooting
    public static final double kFlywheelToleranceMPS = 0.5; // Acceptable tolerance for RPM
    public static final double kFlywheelRadius = .0508; // radius in meters (2 inches)

    public static final double kP = 0.1; // Proportional gain for velocity control
    public static final double kI = 0.0; // Integral gain for velocity control  
    public static final double kD = 0.0; // Derivative gain for velocity control
    public static final double kV = 0.0002; // Velocity feedforward gain
    public static final double kA = 0.0001; // Acceleration feedforward gain
    public static final double kS = 0.05; // Static feedforward gain

    public static final double kStatorCurrentLimit = 0;
    public static final double kSupplyCurrentLimit = 0;
    public static final double kMotionMagicAcceleration = 0;
    public static final double kMotionMagicJerk = 0;

}