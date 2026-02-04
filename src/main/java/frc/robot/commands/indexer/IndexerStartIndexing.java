package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.IndexerSubsystem;

public class IndexerStartIndexing {
    IndexerSubsystem indexerSubsystem;

    public IndexerStartIndexing(IndexerSubsystem indexerSubsystem){
        this.indexerSubsystem = indexerSubsystem;
    }

    public void initialize(){
        indexerSubsystem.setIndexerSpeed(IndexerConstants.indexerSpeed);
    }

    public boolean isFinished(){
        //isFinished always gets checked after execute so this will run once
        return true;
    }
}
