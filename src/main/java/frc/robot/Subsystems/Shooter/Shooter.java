package frc.robot.Subsystems.Shooter;

//two motors velocity, two motors position
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
    
    public ShooterArm(){
        leftMotor = new TalonFX(17, "canivore");
        rightMotor = new TalonFX(18, "canivore");


    }

}
