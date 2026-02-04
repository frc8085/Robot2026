package frc.robot.subsystems.Indexer;

import frc.robot.lib.PIDMotor;
import frc.robot.lib.TalonFXMotor;

public class IndexerSubsystem{
    private PIDMotor indexerMotor;
    private PIDMotor kickerMotor;

    public IndexerSubsystem(){
        this.indexerMotor = new TalonFXMotor(IndexerConstants.CanIndexer);
        this.kickerMotor = new TalonFXMotor(IndexerConstants.CanKicker);
    }

    public void setIndexerSpeed(double speed){
        this.indexerMotor.setSpeed(speed);
    }

    public void setKickerSpeed(double speed){
        this.kickerMotor.setSpeed(speed);
    }
}