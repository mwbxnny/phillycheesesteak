package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//from intake (not otb)
public class Amp extends SubsystemBase{
    private final AmpIO ampIO;
    private AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();
    private double setpointVolts;

    public Amp(AmpIO ampIO){
        this.ampIO = ampIO;
        setpointVolts = 0.0;
    }

    @Override
    public void periodic(){
        ampIO.updateInputs(inputs);
        Logger.processInputs("Amp", inputs);
    }

    public void runAmp(double voltage){
        setpointVolts = voltage;
        ampIO.runAmp(setpointVolts);
    }

    public double getStatorCurrent(){
        return inputs.currentAmps;
    }

    public void updateInputs(AmpIO.AmpIOInputs inputs){
        ampIO.updateInputs(inputs);
    }
}
