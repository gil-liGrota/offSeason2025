package frc.robot.subsystems.CoralArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArm extends SubsystemBase {

    private CoralArmIO io;
    private CoralArmIOInputsAutoLogged inputs = new CoralArmIOInputsAutoLogged();

    public CoralArm(CoralArmIO io) {
        this.io = io;
        // setDefaultCommand(run(() -> io.stayInCurrentGoal()).withName("Default
        // Command"));
        setDefaultCommand(new RepeatCommand(new ConditionalCommand(this.runOnce(() -> io.setVoltage(0)),
                this.runOnce(io::resistGravity), io::isPressed))
                .beforeStarting(new PrintCommand("arm default command")));

        SmartDashboard.putData("Coral Arm", (CoralArmIOReal) io);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral Arm", inputs);
        Logger.recordOutput("Current command",
                this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName());
    }

    public CoralArmIO getIO() {
        return io;
    }

}
