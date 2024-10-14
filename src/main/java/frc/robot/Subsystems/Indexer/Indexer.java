package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private double setpointVolts;

    public Indexer(IndexerIO indexerIO){
        this.indexerIO = indexerIO;
        setpointVolts = 0.0;
    }

    @Override
    public void periodic(){
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Handoff", inputs);
    }

    public void runIndexer(double voltage){
        setpointVolts = voltage;
        indexerIO.runIndexer(setpointVolts);
    }

    public double getStatorCurrent(){
        return inputs.currentAmps;
    }

    public void updateInputs(){
        indexerIO.updateInputs(inputs);
    }
    
}
