package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    
    public Arm(ArmIO armIO){
        this.armIO = armIO;
    }
    @Override
    public void periodic(){
        armIO.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    public void requestArmVoltage(double voltage){
        armIO.setArmVoltage(voltage);
    }

    public void requestSetpoint(double angleDegrees){
    }

    public double getArmStatorCurrent(){
        return inputs.leftArmCurrent;
    }
    
    
}
