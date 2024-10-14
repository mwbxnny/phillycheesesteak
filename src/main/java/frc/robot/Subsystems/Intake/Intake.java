package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//aka otb_intake
public class Intake extends SubsystemBase{
    private final IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double setpointVolts;
    private double pivotSetpoint;

    public Intake(IntakeIO intakeIO){
        this.intakeIO = intakeIO;
        setpointVolts = 0.0;
        pivotSetpoint = 0.0;
    }

    @Override
    public void periodic(){
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void requestPivotVoltage(double voltage){
        intakeIO.setPivotVoltage(voltage);
    }

    public void requestSetpoint(double angleDegrees){
        pivotSetpoint = angleDegrees;
        intakeIO.setPivotPosition(pivotSetpoint);
    }

    public void requestIntakeVoltage(double voltage) {
        setpointVolts = voltage;
        intakeIO.setIntakeVoltage(setpointVolts);
    }

     public void requestIntake(double angleDegrees, double voltage) {
        requestSetpoint(angleDegrees);
        requestIntakeVoltage(voltage);
    }

    public double getStatorCurrent(){
        return inputs.intakeCurrent;
    }

    public double getPivotStatorCurrent(){
        return inputs.pivotCurrent;
    }
}
