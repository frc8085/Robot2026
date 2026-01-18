package frc.robot.lib;

public interface Motor {
    void setSpeed(double speed);

    double getSpeed();
}


/*
 * class motor {
 * PIDMotor driveMotor;
 * PIDMotor armMotor;
 * Motor dumbMotor;
 * 
 * public motor() {
 * this.driveMotor = new KrakenMotor(0);
 * this.armMotor = new SparkMotor(0);
 * this.dumbMotor = new KrakenMotor(1);
 * }
 * 
 * public void setMovement() {
 * this.driveMotor.setMotorPosition(0);
 * }
 * }
 */
