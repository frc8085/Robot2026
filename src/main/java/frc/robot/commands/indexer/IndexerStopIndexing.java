package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer.IndexerSubsystem;

public class IndexerStopIndexing {

    IndexerSubsystem indexerSubsystem;

    public IndexerStopIndexing(IndexerSubsystem indexerSubsystem){
        this.indexerSubsystem = indexerSubsystem;
    }

    public void execute(){
        indexerSubsystem.setIndexerSpeed(0);
    }

    public boolean isFinished(){
        //isFinished always gets checked after execute so this will run once
        return true;
    }
}
