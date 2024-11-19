package frc.robot.Subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//one voltageout req; aka handoff
public class IndexerIOReal implements IndexerIO{
    private final TalonFX IndexerMotor = new TalonFX(15, "canivore");
    private final TalonFXConfiguration IndexerConfigs = new TalonFXConfiguration();

    private VoltageOut IndexerRequest = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Double> current = IndexerMotor.getStatorCurrent();
    private final StatusSignal<Double> temp = IndexerMotor.getDeviceTemp();
    private final StatusSignal<Double> RPS = IndexerMotor.getRotorVelocity();

    private double setpointVolts;

    public IndexerIOReal(){       
        IndexerConfigs.CurrentLimits.StatorCurrentLimit = 50;
        IndexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        IndexerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        IndexerMotor.getConfigurator().apply(IndexerConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temp,
            RPS
        );

        IndexerMotor.optimizeBusUtilization();

        setpointVolts = 0.0;
    }

    public void updateInputs(IndexerIOInputs inputs){
        BaseStatusSignal.refreshAll(
            current,
            temp,
            RPS
        );
        inputs.appliedVolts = IndexerRequest.Output;
        inputs.setpointVolts = this.setpointVolts;
        inputs.currentAmps = current.getValue();
        inputs.temp = temp.getValue();
        inputs.handoffRPS = RPS.getValue();
    }

    public void runIndexer(double output){
        this.setpointVolts = output;
        IndexerMotor.setControl(IndexerRequest.withOutput(output));
    }
}