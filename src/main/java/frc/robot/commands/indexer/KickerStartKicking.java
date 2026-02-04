package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.IndexerSubsystem;

public class KickerStartKicking {

    IndexerSubsystem indexerSubsystem;

    public KickerStartKicking(IndexerSubsystem indexerSubsystem){
        this.indexerSubsystem = indexerSubsystem;
    }

    public void initialize(){
        indexerSubsystem.setKickerSpeed(IndexerConstants.kickerSpeed);
    }

    public boolean isFinished(){
        //isFinished always gets checked after execute so this will run once
        return true;
    }
}
