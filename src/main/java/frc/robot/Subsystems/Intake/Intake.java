package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final TalonFX pivotMotor = new TalonFX(5, "canivore");
    private final TalonFX intakeMotor = new TalonFX(10, "rio");
    private final TalonFXConfigurator pivotConfigurator;
    private final TalonFXConfiguration pivotConfigs;
    private final TalonFXConfigurator intakeConfigurator;
    private final TalonFXConfiguration intakeConfigs;
    MotionMagicVoltage pivotMotorMotionMagicRequest;
    VoltageOut pivotMotorMotionMagicVoltage;
    VoltageOut intakeMotorVoltageRequest;
    double pivotMotorSetpoint;

    private final StatusSignal<Double> pivotCurrent = pivotMotor.getStatorCurrent();
    private final StatusSignal<Double> pivotTemp = pivotMotor.getDeviceTemp();
    private final StatusSignal<Double> pivotRPS = pivotMotor.getRotorVelocity();
    private final StatusSignal<Double> pivotPos = pivotMotor.getRotorPosition();

    private final StatusSignal<Double> intakeCurrent = intakeMotor.getStatorCurrent();
    private final StatusSignal<Double> intakeTemp = intakeMotor.getDeviceTemp();
    private final StatusSignal<Double> intakeRPS = intakeMotor.getRotorVelocity();

    public Intake(){
        pivotConfigurator = pivotMotor.getConfigurator();
        pivotConfigs = new TalonFXConfiguration();
        intakeConfigurator = intakeMotor.getConfigurator();
        intakeConfigs = new TalonFXConfiguration();

        var pivotMotorOutputConfigs = pivotConfigs.MotorOutput;
        pivotMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        pivotMotorOutputConfigs.Inverted = 55;
        
    }




}
