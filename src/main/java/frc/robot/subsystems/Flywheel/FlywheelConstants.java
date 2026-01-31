package frc.robot.subsystems.Flywheel;

public final class FlywheelConstants {
    public static final int kFlywheelMainCanId = 21;
    public static final int kFlywheelFollowerCanID = 22;
    public static final double kFlywheelMaxMPS = 25; // Example max RPM for the flywheel motor
    // public static final double kFlywheelTargetRPM = 4000.0; // Target RPM for shooting
    public static final double kFlywheelToleranceMPS = 0.5; // Acceptable tolerance for RPM
    public static final double kFlywheelRadius = .0508; // radius in meters (2 inches)

    public static final double kP = 0.11; // Proportional gain for velocity control
    public static final double kI = 0.0; // Integral gain for velocity control  
    public static final double kD = 0.0; // Derivative gain for velocity control
    public static final double kV = 0.12; // Velocity feedforward gain
    public static final double kA = 0.01; // Acceleration feedforward gain
    public static final double kS = 0.20; // Static feedforward gain

    public static final double kStatorCurrentLimit = 120;
    public static final double kSupplyCurrentLimit = 70;
    public static final double kSupplyCurrentLowerLimit = 40;
    public static final double kSupplyCurrentLowerTime = 1;
    public static final double kMotionMagicAcceleration = 0;
    public static final double kMotionMagicJerk = 0;

}