package frc.robot.Subsystems.Shooter;

//two motors velocity
public class Shooter {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final StatusSignal<Double> current1;
    private final StatusSignal<Double> temp1;
    private final StatusSignal<Double> RPS1;
    private final StatusSignal<Double> position1;

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    
    public Shooter(){
        leftMotor = new TalonFX(17, "canivore");
        rightMotor = new TalonFX(18, "canivore");
        current1 = leftMotor.getStatorCurrent();
        temp1 = leftMotor.getDeviceTemp();
        RPS1 = leftMotor.getRotorVelocity(); //rotations per sec
        position1 = leftMotor.getPosition();

        var leftMotorConfigs = new TalonFXConfiguration();
        leftMotorConfigs.CurrentLimtis.StatorCurrentLimit = 0.0;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotorConfigs.Slot0.kP = 0.068419;
        leftMotorConfigs.Slot0.kI = 0.0;
        leftMotorConfigs.Slot0.kD = 0.0;
        leftMotorConfigs.Slot0.kS = 0.16488;
        leftMotorConfigs.Slot0.kV = 0.11167;
        leftMotorConfigs.Slot0.kA = 0.0077173;

        var rightMotorConfigs = new TalonFXConfiguration();
        rightMotorConfigs.CurrentLimits.StatorCurrentLimit = Constants.shooterConstants.statorCurrentLimit;
        rightMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfigs.Slot0.kP = 0.068419;
        rightMotorConfigs.Slot0.kI = 0.0;
        rightMotorConfigs.Slot0.kD = 0.0;
        rightMotorConfigs.Slot0.kS = 0.16488;
        rightMotorConfigs.Slot0.kV = 0.11167;
        rightMotorConfigs.Slot0.kA = 0.0077173;

        leftMotor.getConfigurator().apply(leftMotorConfigs);
        rightMotor.getConfigurator().apply(rightMotorConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current1,
            temp1,
            RPS1,
            position1
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    public void setVelocity(double velocity, double ratio){
        leftMotor.setControl(leftRequestVelocity.withVelocity(velocity*1.0/(Unit.inchesToMeters(4)*Math.PI)));
        rightMotor.setControl(rightRequestVelocity.withVelocity((velocity*ratio)*1.0/(Unit.inchesToMeters(4)*Math.PI)));
    }

    public void SetVOltage(double voltage){
        rightMotor.setControl(new FOllower(leftMotor.getDeviceID(), true));
        leftMotor.setControl(shootRequestVoltage.withOutput(voltage));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current1, temp1, RPS1);
        SmartDashboard.putNumber("Shooter Current", current1.getValue());
        SmartDashboard.putNumber("Shooter Temperature", temp1.getValue());
        SmartDashboard.putNumber("Shooter Speed (RPS)", RPS1.getValue());
    }
}