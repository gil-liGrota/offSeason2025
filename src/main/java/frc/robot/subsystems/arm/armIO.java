package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface armIO {

    @AutoLog
    public static class armIOInputs {
        boolean armConnected = false;
        double armVelocity = 0.0;
        double armPosition = 0.0;
        double armAppliedVolts = 0.0;
        boolean foldSwitch = false;
    }

    public default void updateInputs(armIOInputs inputs) {
    }

    public default void setSpeed(double speed) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setSetpoint(double goal) {
    }//

    public default BooleanSupplier atGoal() {
        return () -> false;
    }

    public default void stopMotor() {
    }

    public default void resetIfPressed() {
    }

    public default boolean isPressed() {
        return false;
    }

    public default double getPosition() {
        return 0;
    }

    public default void resetPID() {
    }//

    public default void resetPID(double newGoal) {
    }//
}
