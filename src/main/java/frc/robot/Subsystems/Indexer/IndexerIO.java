package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs{
        public double appliedVolts = 0.0;
        public double setpointVolts = 0.0;
        public double currentAmps = 0.0;
        public double temp = 0.0;
        public double handoffRPS = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs){}

    public default void runIndexer(double volts){}
}
