package frc.robot.lib;

public interface AbsoluteEncoder {
    double getPositionAbsolute();

    void setPositionAbsolute(double reference);
}