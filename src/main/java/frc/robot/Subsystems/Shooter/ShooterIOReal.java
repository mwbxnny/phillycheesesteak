package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Commons.Conversions;
import frc.robot.Constants.shooterConstants;

//two motors velocity
public class ShooterIOReal implements ShooterIO{
    private final TalonFX leftShooterMotor  = new TalonFX(16, "rio");
    private final TalonFX rightShooterMotor = new TalonFX(17, "rio");
    private TalonFXConfiguration leftShooterMotorConfigs = new TalonFXConfiguration();
    private TalonFXConfiguration rightShooterMotorConfigs = new TalonFXConfiguration();

    private final StatusSignal<Double> leftShooterCurrent = leftShooterMotor.getStatorCurrent();
    private final StatusSignal<Double> leftShooterTemp = leftShooterMotor.getDeviceTemp();
    private final StatusSignal<Double> leftShooterRPS = leftShooterMotor.getRotorVelocity(); //rotations per sec
    private final StatusSignal<Double> rightShooterCurrent = leftShooterMotor.getStatorCurrent();
    private final StatusSignal<Double> rightShooterTemp = leftShooterMotor.getDeviceTemp();
    private final StatusSignal<Double> rightShooterRPS = leftShooterMotor.getRotorVelocity(); //rotations per sec
    

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);

    private double leftShooterSetpointMPS;
    private double rightShooterSetpointMPS;

    public ShooterIOReal(){
        leftShooterMotorConfigs.CurrentLimits.StatorCurrentLimit = 50.0;
        leftShooterMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftShooterMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftShooterMotorConfigs.Slot0.kP = 0.775; //nums from 2024 offseason code
        leftShooterMotorConfigs.Slot0.kI = 0.0;
        leftShooterMotorConfigs.Slot0.kD = 0.0;
        leftShooterMotorConfigs.Slot0.kS = 0.4;
        leftShooterMotorConfigs.Slot0.kV = 0.153;
        leftShooterMotorConfigs.Slot0.kA = 0.0;

        rightShooterMotorConfigs.CurrentLimits.StatorCurrentLimit = 50.0;
        rightShooterMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightShooterMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightShooterMotorConfigs.Slot0.kP = 0.775;
        rightShooterMotorConfigs.Slot0.kI = 0.0;
        rightShooterMotorConfigs.Slot0.kD = 0.0;
        rightShooterMotorConfigs.Slot0.kS = 0.4;
        rightShooterMotorConfigs.Slot0.kV = 0.153;
        rightShooterMotorConfigs.Slot0.kA = 0.0;

        leftShooterMotor.getConfigurator().apply(leftShooterMotorConfigs);
        rightShooterMotor.getConfigurator().apply(rightShooterMotorConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterRPS,
            rightShooterRPS
        );

        leftShooterMotor.optimizeBusUtilization();
        rightShooterMotor.optimizeBusUtilization();

        leftShooterSetpointMPS = 0;
        rightShooterSetpointMPS = 0;
    }

    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterRPS,
            rightShooterRPS
        );

        inputs.appliedVolts = shootRequestVoltage.Output;
        inputs.currentAmps = new double[] { leftShooterCurrent.getValue(),
                rightShooterCurrent.getValue() };
        inputs.temp = new double[] { leftShooterTemp.getValue(),
                rightShooterTemp.getValue() };
        inputs.shooterVelMPS = new double[] {Conversions.RPStoMPS(leftShooterRPS.getValue(), shooterConstants.wheelCircumferenceMeters, 1), Conversions.RPStoMPS(rightShooterRPS.getValue(), shooterConstants.wheelCircumferenceMeters, 1)};
        inputs.shooterSetpointsMPS = new double[] {leftShooterSetpointMPS, rightShooterSetpointMPS};
    }

    public void setVelocity(double velocity, double ratio){
        this.leftShooterSetpointMPS = velocity;
        this.rightShooterSetpointMPS = velocity * ratio;
        leftShooterMotor.setControl(leftRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity, shooterConstants.wheelCircumferenceMeters, 1)));
        rightShooterMotor.setControl(rightRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity * ratio, shooterConstants.wheelCircumferenceMeters, 1)));
    }

    public void zeroVelocity(){
        this.leftShooterSetpointMPS = 0;
        this.rightShooterSetpointMPS = 0;
        leftShooterMotor.setControl(leftRequestVelocity.withVelocity(0));
        rightShooterMotor.setControl(rightRequestVelocity.withVelocity(0));
    }

    
}