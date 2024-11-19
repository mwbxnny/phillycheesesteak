package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.AutoLog;

public interface AmpIO {
    @AutoLog
    public static class AmpIOInputs{
        public double appliedVolts = 0.0;
        public double setpointVolts = 0.0;
        public double currentAmps = 0.0;
        public double temp = 0.0;
        public double handoffRPS = 0.0;
    }

    public default void updateInputs(AmpIOInputs inputs){
    }

    public default void runAmp(double volts){
    }
}