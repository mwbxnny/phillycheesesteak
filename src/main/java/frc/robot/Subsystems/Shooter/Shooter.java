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

//two motors velocity
public class Shooter extends SubsystemBase{
    private final TalonFX leftShooterMotor;
    private final TalonFX rightShooterMotor;
    private final StatusSignal<Double> current1;
    private final StatusSignal<Double> temp1;
    private final StatusSignal<Double> RPS1;
    private final StatusSignal<Double> position1;

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);

    public Shooter(){
        leftShooterMotor = new TalonFX(16, "canivore"); //number may be wrong
        rightShooterMotor = new TalonFX(17, "canivore"); //number may be wrong
        current1 = leftShooterMotor.getStatorCurrent();
        temp1 = leftShooterMotor.getDeviceTemp();
        RPS1 = leftShooterMotor.getRotorVelocity(); //rotations per sec
        position1 = leftShooterMotor.getPosition();

        var leftShooterMotorConfigs = new TalonFXConfiguration();
        leftShooterMotorConfigs.CurrentLimits.StatorCurrentLimit = 0.0;
        leftShooterMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftShooterMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftShooterMotorConfigs.Slot0.kP = 0.068419;
        leftShooterMotorConfigs.Slot0.kI = 0.0;
        leftShooterMotorConfigs.Slot0.kD = 0.0;
        leftShooterMotorConfigs.Slot0.kS = 0.16488;
        leftShooterMotorConfigs.Slot0.kV = 0.11167;
        leftShooterMotorConfigs.Slot0.kA = 0.0077173;

        var rightShooterMotorConfigs = new TalonFXConfiguration();
        rightShooterMotorConfigs.CurrentLimits.StatorCurrentLimit = 0.0;
        rightShooterMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightShooterMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightShooterMotorConfigs.Slot0.kP = 0.068419;
        rightShooterMotorConfigs.Slot0.kI = 0.0;
        rightShooterMotorConfigs.Slot0.kD = 0.0;
        rightShooterMotorConfigs.Slot0.kS = 0.16488;
        rightShooterMotorConfigs.Slot0.kV = 0.11167;
        rightShooterMotorConfigs.Slot0.kA = 0.0077173;

        leftShooterMotor.getConfigurator().apply(leftShooterMotorConfigs);
        rightShooterMotor.getConfigurator().apply(rightShooterMotorConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current1,
            temp1,
            RPS1,
            position1
        );

        leftShooterMotor.optimizeBusUtilization();
        rightShooterMotor.optimizeBusUtilization();
    }

    public void setVelocity(double velocity, double ratio){
        leftShooterMotor.setControl(leftRequestVelocity.withVelocity(velocity*1.0/(Units.inchesToMeters(4)*Math.PI)));
        rightShooterMotor.setControl(rightRequestVelocity.withVelocity((velocity*ratio)*1.0/(Units.inchesToMeters(4)*Math.PI)));
    }

    public void SetVoltage(double voltage){
        rightShooterMotor.setControl(new Follower(leftShooterMotor.getDeviceID(), true));
        leftShooterMotor.setControl(shootRequestVoltage.withOutput(voltage));
    }

    //@Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current1, temp1, RPS1);
        SmartDashboard.putNumber("Shooter Current", current1.getValue());
        SmartDashboard.putNumber("Shooter Temperature", temp1.getValue());
        SmartDashboard.putNumber("Shooter Speed (RPS)", RPS1.getValue());
    }
}