package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//two motors: position
public class Arm extends SubsystemBase{
    private final TalonFX leftArm = new TalonFX(18, "rio");
    private final TalonFXConfigurator leftArmConfigurator;
    private final TalonFXConfiguration leftArmConfigs;
    MotionMagicVoltage leftArmMotionMagicRequest;
    VoltageOut leftArmVoltageRequest;
    double leftArmSetpoint;

    private final TalonFX rightArm = new TalonFX(19, "rio");
    private final TalonFXConfigurator rightArmConfigurator;
    private final TalonFXConfiguration rightArmConfigs;
    MotionMagicVoltage rightArmMotionMagicRequest;
    VoltageOut rightArmVoltageRequest;
    double rightArmSetpoint;

    private final StatusSignal<Double> leftArmCurrent = leftArm.getStatorCurrent();
    private final StatusSignal<Double> leftArmTemp = leftArm.getDeviceTemp();
    private final StatusSignal<Double> leftArmRPS = leftArm.getRotorVelocity();
    private final StatusSignal<Double> leftArmPos = leftArm.getRotorPosition();

    private final StatusSignal<Double> rightArmCurrent = leftArm.getStatorCurrent();
    private final StatusSignal<Double> rightArmTemp = leftArm.getDeviceTemp();
    private final StatusSignal<Double> rightArmRPS = leftArm.getRotorVelocity();
    private final StatusSignal<Double> rightArmPos = leftArm.getRotorPosition();

    public Arm(){
        leftArmConfigurator = leftArm.getConfigurator();
        leftArmConfigs = new TalonFXConfiguration();
        rightArmConfigurator = rightArm.getConfigurator();
        rightArmConfigs = new TalonFXConfiguration();

        var leftArmMotorOutputConfigs = leftArmConfigs.MotorOutput;
        leftArmMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        leftArmMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        var leftArmCurrentLimitConfigs = leftArmConfigs.CurrentLimits;
        leftArmCurrentLimitConfigs.StatorCurrentLimit = 50;
        leftArmCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        var rightArmMotorOutputConfigs = leftArmConfigs.MotorOutput;
        rightArmMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        rightArmMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        var rightArmCurrentLimitConfigs = rightArmConfigs.CurrentLimits;
        rightArmCurrentLimitConfigs.StatorCurrentLimit = 50;
        rightArmCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        leftArmConfigs.Slot0.kP = 10;
        leftArmConfigs.Slot0.kI = 0.0;
        leftArmConfigs.Slot0.kD = 0.0;
        leftArmConfigs.Slot0.kS = 0.23;
        leftArmConfigs.Slot0.kV = 0.25;
        leftArmConfigs.Slot0.kA = 0.010154;
        leftArmConfigs.Slot0.kG = 0.27;
        leftArmConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        rightArmConfigs.Slot0.kP = 10;
        rightArmConfigs.Slot0.kI = 0.0;
        rightArmConfigs.Slot0.kD = 0.0;
        rightArmConfigs.Slot0.kS = 0.23;
        rightArmConfigs.Slot0.kV = 0.25;
        rightArmConfigs.Slot0.kA = 0.010154;
        rightArmConfigs.Slot0.kG = 0.27;
        rightArmConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        leftArmConfigs.MotionMagic.MotionMagicCruiseVelocity = 75;
        leftArmConfigs.MotionMagic.MotionMagicAcceleration = 150;
        leftArmConfigs.MotionMagic.MotionMagicJerk = 10000;

        rightArmConfigs.MotionMagic.MotionMagicCruiseVelocity = 75;
        rightArmConfigs.MotionMagic.MotionMagicAcceleration = 150;
        rightArmConfigs.MotionMagic.MotionMagicJerk = 10000;

        leftArmMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        leftArmVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        rightArmMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        rightArmVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        leftArm.setPosition(0);
        rightArm.setPosition(0);

        rightArm.setControl(new Follower(leftArm.getDeviceID(), true));
        leftArmConfigurator.apply(leftArmConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftArmCurrent,
            leftArmTemp,
            leftArmRPS,
            leftArmPos,
            rightArmCurrent,
            rightArmTemp,
            rightArmRPS,
            rightArmPos
        );
    }

    public void requestArmVoltage(double voltage){
        leftArm.setControl(leftArmVoltageRequest.withOutput(voltage));
    }

    public void requestSetpoint(double angleDegrees){
        leftArmSetpoint = angleDegrees;
        double leftArmSetpointRotations = angleDegrees / (360.0 / (97.337962963));
        leftArm.setControl(leftArmMotionMagicRequest.withPosition(leftArmSetpointRotations));
    }

    public void zeroPosition(){
        leftArm.setPosition(0);
    }

    @Override
    public void periodic() {
    SmartDashboard.putNumber("Arm Current", leftArmCurrent.getValue());
    SmartDashboard.putNumber("Arm Temperature", leftArmTemp.getValue());
    SmartDashboard.putNumber("Arm Speed (RPS)", leftArmRPS.getValue());
    }
}