package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//one pivot (pos), one roller/intake (vel)
public class Intake extends SubsystemBase{
    private final TalonFX pivotMotor = new TalonFX(13, "canivore");
    private final TalonFX intakeMotor = new TalonFX(14, "rio");
    private final TalonFXConfigurator pivotConfigurator;
    private final TalonFXConfiguration pivotConfigs;
    private final TalonFXConfigurator intakeConfigurator;
    private final TalonFXConfiguration intakeConfigs;
    MotionMagicVoltage pivotMotorMotionMagicRequest;
    VoltageOut pivotMotorVoltageRequest;
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
        pivotMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        var pivotCurrentLimitConfigs = pivotConfigs.CurrentLimits;
        pivotCurrentLimitConfigs.StatorCurrentLimit = 50;
        pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        var slot0Configs = pivotConfigs.Slot0;
        slot0Configs.kP = 6.5;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kS = 0.169; 
        slot0Configs.kV = 0.0649; 
        slot0Configs.kA = 0.0246;
        slot0Configs.kG = 0.0301;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = pivotConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 60;
        motionMagicConfigs.MotionMagicAcceleration = 120;
        motionMagicConfigs.MotionMagicJerk = 10000;

        var intakeMotorOutputConfigs = intakeConfigs.MotorOutput;
        intakeMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        intakeMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        var intakeCurrentLimitConfigs = intakeConfigs.CurrentLimits;
        intakeCurrentLimitConfigs.StatorCurrentLimit = 50;
        intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        pivotMotorMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        pivotMotorVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        intakeMotorVoltageRequest =  new VoltageOut(0).withEnableFOC(true);

        pivotMotor.setPosition(0);

        pivotConfigurator.apply(pivotConfigs);
        intakeConfigurator.apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            pivotCurrent,
            pivotPos,
            pivotRPS,
            pivotTemp
        );

        intakeMotor.optimizeBusUtilization();
    }

    public void requestPivotVoltage(double voltage){
        pivotMotor.setControl(pivotMotorVoltageRequest.withOutput(voltage));

    }

    public void requestSetpoint(double angleDegrees){
        pivotMotorSetpoint = angleDegrees;
        double pivotSetpointRotations = angleDegrees / (360.0/ (23.625));
        pivotMotor.setControl(pivotMotorMotionMagicRequest.withPosition(pivotSetpointRotations));
        
    }

    public void requestIntakeVoltage(double voltage){
        intakeMotor.setControl(intakeMotorVoltageRequest.withOutput(voltage));
    }

    public void requestIntake(double angleDegrees, double voltage) {
        requestSetpoint(angleDegrees);
        requestIntakeVoltage(voltage);
    }

    public void zeroPosition(){
        pivotMotor.setPosition(0);
    }

    @Override
    public void periodic() {
    BaseStatusSignal.refreshAll(intakeCurrent, intakeTemp, intakeRPS);
    SmartDashboard.putNumber("Intake Current", intakeCurrent.getValue());
    SmartDashboard.putNumber("Intake Temperature", intakeTemp.getValue());
    SmartDashboard.putNumber("Intake Speed (RPS)", intakeRPS.getValue());
    SmartDashboard.putNumber("Pivot Current", pivotCurrent.getValue());
    SmartDashboard.putNumber("Pivot Temperature", pivotTemp.getValue());
    SmartDashboard.putNumber("Pivot Speed (RPS)", pivotRPS.getValue());
    }
}