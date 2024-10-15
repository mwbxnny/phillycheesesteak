package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public double leftArmCurrent = 0;
        public double leftArmTemp = 0;
        public double leftArmRPS = 0;
        public double leftArmPos = 0;
        public double rightArmCurrent = 0;
        public double rightArmTemp = 0;
        public double rightArmRPS = 0;
        public double rightArmPos = 0;
    }

    public void updateInputs(ArmIOInputs inputs);
    public void setArmVoltage(double voltage);
    public void setSetpoint(double angleDegrees);
    public void zeroPosition();
}
