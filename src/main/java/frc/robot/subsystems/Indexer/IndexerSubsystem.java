package frc.robot.subsystems.Indexer;

import frc.robot.lib.PIDMotor;
import frc.robot.lib.TalonFXMotor;

public class IndexerSubsystem{
    private PIDMotor indexerMotor;

    public IndexerSubsystem(){
        this.indexerMotor = new TalonFXMotor(IndexerConstants.CanIndexer);
    }

    public void setSpeed(double speed){
        this.indexerMotor.setSpeed(speed);
    }
}