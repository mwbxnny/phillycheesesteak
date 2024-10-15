package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrent = 0.0;
        public double pivotSetpointDeg = 0.0;
        public double pivotSetpointRot = 0.0;
        public double pivotPosDeg = 0.0;
        public double pivotPosRot = 0.0;
        public double pivotTemperature = 0.0;
        public double pivotRPS = 0.0;

        public double intakeTemp = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrent = 0.0;
        public double intakeRPS = 0.0;
        public double intakeSetpointVolts = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs){}

    public default void setPivotVoltage(double voltage){}

    public default void setPivotPosition(double angleDegrees){}

    public default void setIntakeVoltage(double voltage){}

    public default void zeroPosition(){}
}
