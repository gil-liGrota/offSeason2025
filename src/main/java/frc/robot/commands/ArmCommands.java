package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.arm;

public class ArmCommands {
    public Command SetVoltage(arm arm, double voltage){
        return Commands.run(() -> arm.getIO().setVoltage(voltage), arm);
    }
}
