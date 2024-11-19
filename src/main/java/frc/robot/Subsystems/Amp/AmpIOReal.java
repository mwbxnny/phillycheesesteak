package frc.robot.Subsystems.Amp;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

//voltageout req, one motor
public class AmpIOReal implements AmpIO {
    private final TalonFX amp = new TalonFX(20, "rio");
    private final TalonFXConfiguration ampConfigs = new TalonFXConfiguration();
    
    private VoltageOut ampRequest = new VoltageOut(0).withEnableFOC(true);
    
    private final StatusSignal<Double> current = amp.getStatorCurrent();
    private final StatusSignal<Double> temp = amp.getDeviceTemp();
    private final StatusSignal<Double> RPS = amp.getRotorVelocity();
    
    private double setpointVolts;

    public AmpIOReal(){
        ampConfigs.CurrentLimits.StatorCurrentLimit = 50; //number might be wrong
        ampConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        ampConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        amp.getConfigurator().apply(ampConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temp,
            RPS
        );

        amp.optimizeBusUtilization();

        setpointVolts = 0.0;
    }

    public void updateInputs(AmpIOInputs inputs){
        BaseStatusSignal.refreshAll(
            current,
            temp,
            RPS
        );
        
        inputs.appliedVolts = ampRequest.Output;
        inputs.setpointVolts = this.setpointVolts;
        inputs.currentAmps = current.getValue();
        inputs.temp = temp.getValue();
        inputs.handoffRPS = RPS.getValue();
    }

    public void runAmp(double voltage){
        this.setpointVolts = voltage;
        amp.setControl(ampRequest.withOutput(voltage));
    }
}
