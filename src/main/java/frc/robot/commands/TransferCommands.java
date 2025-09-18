package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.robot.subsystems.Transfer.TransferConstants.*;

import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Transfer.Transfer;

public class TransferCommands {

    public static Command coralintake(Transfer transfer, double voltage) {
        return Commands.startEnd(() -> transfer.getIO().setVoltage(voltage),
                () -> transfer.getIO().stopMotor(), transfer);
    }

    public static Command riffOutake(Transfer transfer, Elevator elevator) {
        double voltage;
        if (elevator.getIO().getPosition() > 20) {
            voltage = -12;
        } else {
            voltage = 12;
        }

        return Commands.startEnd(() -> transfer.getIO().setVoltage(voltage),
                () -> transfer.getIO().stopMotor(), transfer);
    }

    // public static Command autoIntakeCoral(Transfer transfer) {
    // return Commands.startEnd(() -> transfer.getIO().setVoltage(5),
    // () -> transfer.getIO().stopMotor(),
    // transfer).until(transfer.getIO()::isCoralIn);
    // }
}
