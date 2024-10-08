package frc.robot.Subsystems.Amp;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//voltageout req, one motor
public class Amp extends SubsystemBase{
    private final TalonFX AmpMotor;
    private VoltageOut AmpRequest;
    private final StatusSignal<Double> current;
    private final StatusSignal<Double> temp;
    private final StatusSignal<Double> RPS;
    private double setpointVolts;

    public Amp(){
        AmpMotor = new TalonFX(20, "rio");
        AmpRequest = new VoltageOut(0).withEnableFOC(true);
        current = AmpMotor.getStatorCurrent();
        temp = AmpMotor.getDeviceTemp();
        RPS = AmpMotor.getRotorVelocity();

        var AmpConfigs = new TalonFXConfiguration();
        var AmpCurrentLimitConfigs = AmpConfigs.CurrentLimits;
        AmpCurrentLimitConfigs.StatorCurrentLimit = 50; //number might be wrong
        AmpCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        AmpConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        AmpMotor.getConfigurator().apply(AmpConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temp,
            RPS
        );

        AmpMotor.optimizeBusUtilization();

        setpointVolts = 0.0;
    }

    public void runAmp(double voltage){
        setpointVolts = voltage;
        AmpMotor.setControl(AmpRequest.withOutput(voltage));
    }

    public double getStatorCurrent(){
        return current.getValue();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current, temp, RPS);
        SmartDashboard.putNumber("Amp Voltage", setpointVolts);
        SmartDashboard.putNumber("Amp Current", current.getValue());
        SmartDashboard.putNumber("Amp Temperature", temp.getValue());
        SmartDashboard.putNumber("Amp Speed (RPS)", RPS.getValue());
    }
}
