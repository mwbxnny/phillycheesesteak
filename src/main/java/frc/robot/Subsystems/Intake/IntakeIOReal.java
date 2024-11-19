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

import frc.Commons.Conversions;

//one pivot (pos), one roller/intake (vel)
public class IntakeIOReal implements IntakeIO{
    private final TalonFX pivot = new TalonFX(13, "canivore");
    private final TalonFX intake = new TalonFX(14, "canivore");
    private final TalonFXConfigurator pivotConfigurator = pivot.getConfigurator();
    private final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
    private final TalonFXConfigurator intakeConfigurator = intake.getConfigurator();
    private final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    MotionMagicVoltage pivotMotionMagicRequest;
    VoltageOut pivotVoltageRequest;
    VoltageOut intakeVoltageRequest;

    private final StatusSignal<Double> pivotCurrent = pivot.getStatorCurrent();
    private final StatusSignal<Double> pivotTemp = pivot.getDeviceTemp();
    private final StatusSignal<Double> pivotRPS = pivot.getRotorVelocity();
    private final StatusSignal<Double> pivotPos = pivot.getRotorPosition();

    private final StatusSignal<Double> intakeCurrent = intake.getStatorCurrent();
    private final StatusSignal<Double> intakeTemp = intake.getDeviceTemp();
    private final StatusSignal<Double> intakeRPS = intake.getRotorVelocity();

    private double pivotSetpoint;
    private double intakeSetpointVolts;

    public IntakeIOReal(){ 
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfigs.CurrentLimits.StatorCurrentLimit = 50;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfigs.CurrentLimits.StatorCurrentLimit = 50;
        intakeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        var slot0Configs = pivotConfigs.Slot0;
        slot0Configs.kP = 6.5;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kS = 0.169; 
        slot0Configs.kV = 0.0649; 
        slot0Configs.kA = 0.0246;
        slot0Configs.kG = 0.0301;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = 60;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = 120;
        pivotConfigs.MotionMagic.MotionMagicJerk = 10000;

        pivot.setPosition(0);

        pivotConfigurator.apply(pivotConfigs);
        intakeConfigurator.apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            pivotCurrent,
            pivotPos,
            pivotRPS,
            pivotTemp,
            intakeTemp,
            intakeCurrent,
            intakeRPS
        );

        intake.optimizeBusUtilization();
        pivot.optimizeBusUtilization();
        
        pivotSetpoint = 0;
        intakeSetpointVolts = 0;
    }

    @Override
    public void setPivotVoltage(double voltage){
        pivot.setControl(pivotVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setPivotPosition(double angleDegrees) {
        this.pivotSetpoint = angleDegrees;
        double pivotSetpointRotations = Conversions.DegreesToRotations(angleDegrees, 76.1904761905); //offseason num
        pivot.setControl(pivotMotionMagicRequest.withPosition(pivotSetpointRotations));
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        this.intakeSetpointVolts = voltage;
        intake.setControl(intakeVoltageRequest.withOutput(intakeSetpointVolts));
    }

    @Override
    public void zeroPosition() {
        pivot.setPosition(0);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                pivotCurrent,
                pivotPos,
                pivotRPS,
                pivotTemp,
                intakeCurrent,
                intakeTemp,
                intakeRPS
        );
        inputs.intakeCurrent = intakeCurrent.getValue();
        inputs.intakeTemp = intakeTemp.getValue();
        inputs.intakeRPS = intakeRPS.getValue();
        inputs.pivotCurrent = pivotCurrent.getValue();
        inputs.pivotTemperature = pivotTemp.getValue();
        inputs.pivotRPS = pivotRPS.getValue();
        inputs.intakeSetpointVolts = this.intakeSetpointVolts;
        inputs.pivotSetpointDeg = this.pivotSetpoint;
    }
}