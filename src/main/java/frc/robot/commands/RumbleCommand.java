package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;

public class RumbleCommand extends Command {
    private final XboxController controller;
    private final double duration;
    private final double intensity;
    private double startTime;

    public RumbleCommand(XboxController controller, double duration, double intensity) {
        this.controller = controller;
        this.duration = duration;
        this.intensity = intensity;
    }

    @Override
    public void initialize() {
        controller.setRumble(RumbleType.kLeftRumble, intensity);
        controller.setRumble(RumbleType.kRightRumble, intensity);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kLeftRumble, 0.0);
        controller.setRumble(RumbleType.kRightRumble, 0.0);
    }
}
