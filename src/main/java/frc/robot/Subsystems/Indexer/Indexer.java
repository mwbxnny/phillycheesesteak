package frc.robot.Subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//one voltageout req
public class Indexer extends SubsystemBase{
    private final TalonFX IndexerMotor;
    private VoltageOut IndexerRequest;
    private final StatusSignal<Double> current;
    private final StatusSignal<Double> temp;
    private final StatusSignal<Double> RPS;
    private double setpointVolts;

    public Indexer(){
        IndexerMotor = new TalonFX(15, "canivore");
        IndexerRequest = new VoltageOut(0).withEnableFOC(true);
        current = IndexerMotor.getStatorCurrent();
        temp = IndexerMotor.getDeviceTemp();
        RPS = IndexerMotor.getRotorVelocity();

        var IndexerConfigs = new TalonFXConfiguration();
        var IndexerCurrentLimitConfigs = IndexerConfigs.CurrentLimits;
        IndexerCurrentLimitConfigs.StatorCurrentLimit = 50;
        IndexerCurrentLimitConfigs.StatorCurrentLimitEnable = true;
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

    public void runIndexer(double voltage){
        setpointVolts = voltage;
        IndexerMotor.setControl(IndexerRequest.withOutput(voltage));
    }

    public double getStatorCurrent(){
        return current.getValue();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current, temp, RPS);
        SmartDashboard.putNumber("Handoff Voltage", setpointVolts);
        SmartDashboard.putNumber("Handoff Current", current.getValue());
        SmartDashboard.putNumber("Handoff Temperature", temp.getValue());
        SmartDashboard.putNumber("Handoff Speed (RPS)", RPS.getValue());
    }
}
