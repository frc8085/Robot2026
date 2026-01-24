package frc.robot.subsystems.Climber;

public class ClimberConstants {
    

    public static final int kCanL1Climb = 21; //deploy motor
    public static final int kCanL2Climb = 22; //main up/down motor
    //These are for deploy motor
    public static final double kL1ClimbMinRot = 0; 
    public static final double kL1ClimbMaxRot = 10;
    public static final double kL1ClimbMinRaiseRot = 0;
    //These are for main climb motor
    public static final double kMaxClimbMotorVelocity = 0; //basically just climb motor speed
    public static final double kMinClimbMotorPosition = 0; 
    public static final double kMaxClimbMotorPosition = 0;
    public static final double kClimbMotorClimbPosition = 0; //bad name. This is the climb position, meaning it pulls the robot up as it goes to this position.

    public class L1ClimberPID {
        public static final double P = 1;
        public static final double I = 0;
        public static final double D = 0;
        public static final double FF = 0;
    }

    public class Slot0Configs {
        public static final double kP = 0.12;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kS = 0.20;
        public static final double kMaxSpeed = 80;
    }

}
