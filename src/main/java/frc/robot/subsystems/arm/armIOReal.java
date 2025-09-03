package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.armConstants.ARM_ID;
import static frc.robot.subsystems.arm.armConstants.CURRENT_LIMIT;
import static frc.robot.subsystems.arm.armConstants.FOLD_SWITCH_ID;
import static frc.robot.subsystems.arm.armConstants.INVERTED;
import static frc.robot.subsystems.arm.armConstants.KA;
import static frc.robot.subsystems.arm.armConstants.KD;
import static frc.robot.subsystems.arm.armConstants.KG;
import static frc.robot.subsystems.arm.armConstants.KI;
import static frc.robot.subsystems.arm.armConstants.KP;
import static frc.robot.subsystems.arm.armConstants.KS;
import static frc.robot.subsystems.arm.armConstants.KV;
import static frc.robot.subsystems.arm.armConstants.POSITION_CONVERSION_FACTOR;
import static frc.robot.subsystems.arm.armConstants.VOLTAGE_COMPENSATION;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class armIOReal implements armIO {
    POMSparkMax motor;
    RelativeEncoder encoder;
    PIDController pidController;
    private POMDigitalInput foldSwitch;
    private ArmFeedforward ff;

    public armIOReal() {
        motor = new POMSparkMax(ARM_ID);
        encoder = motor.getEncoder();
        foldSwitch = new POMDigitalInput(FOLD_SWITCH_ID);
        pidController = new PIDController(KP, KI, KD);
        ff = new ArmFeedforward(KS, KG, KV, KA);

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast).inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);
        // encoder.getPosition(0); to the pid

    }

    @Override
    public void updateInputs(armIOInputs inputs) {
        inputs.armConnected = true;
        inputs.armVelocity = encoder.getVelocity();
        inputs.armPosition = encoder.getPosition();
        inputs.armAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.foldSwitch = foldSwitch.get();

        resetIfPressed();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setSetpoint(double goal) {
        pidController.setSetpoint(goal);
        setVoltage(ff.calculate(encoder.getPosition()/* TODO change to position rad */, encoder.getVelocity())
                + pidController.calculate(getPosition()));
    }

    @Override
    public void setFeedForward(double velocity) {
        double voltage = ff.calculate(encoder.getPosition()/* change to position rad */, velocity);
        motor.setVoltage((voltage));
    }

    @Override
    public BooleanSupplier atGoal() {
        return () -> pidController.atSetpoint();
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public boolean isPressed() {
        return foldSwitch.get();
    }

    @Override
    public void resetIfPressed() {
        if (isPressed()) {
            encoder.setPosition(0);
        }
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void resetPID() {
        pidController.reset();
    }

}
