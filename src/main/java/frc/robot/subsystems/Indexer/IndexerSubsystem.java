package frc.robot.subsystems.Indexer;

import frc.robot.lib.PIDMotor;
import frc.robot.lib.SparkMotor;

public class IndexerSubsystem{
    private PIDMotor indexerMotor;

    public IndexerSubsystem(){
        this.indexerMotor = new SparkMotor(IndexerConstants.kIndexerMotorCanId);
    }

    public void setSpeed(double speed){
        this.indexerMotor.setSpeed(speed);
    }
}