package frc.robot.commands;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.subsystems.CoralArm.CoralArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm.CoralArm;
import frc.robot.subsystems.Elevator.Elevator;

public class RiffCommands {
    ElevatorCommands elevatorCommands = new ElevatorCommands();
    CoralArmCommands armCommands = new CoralArmCommands();

    public Command L1(Elevator elevator, CoralArm arm) {
        return ElevatorCommands.goToPosition(elevator, 16.7);
    }

    public Command L2(Elevator elevator, CoralArm arm) {
        return ElevatorCommands.goToPosition(elevator,
                L4_ELEVATOR_POSITION)
                .until(() -> elevator.getIO().getPosition() - 0.4 >= L2_ELEVATOR_POSITION)
                .andThen(CoralArmCommands.goToPosition(arm, L2_ARM_POSITION));
    }

    public Command L3(Elevator elevator, CoralArm arm) {
        return CoralArmCommands.goToPosition(arm, L4_ARM_POSITION)
                // .andThen(ElevatorCommands.closeElevator(elevator)
                .andThen(CoralArmCommands.goToPosition(arm, 1.1));
    }

    public Command L4(Elevator elevator, CoralArm arm) {
        return ElevatorCommands.goToPosition(elevator, L4_ELEVATOR_POSITION)
                .until(() -> elevator.getIO().getPosition() - 0.4 >= L4_ELEVATOR_POSITION)
                .alongWith(CoralArmCommands.goToPosition(arm, L4_ARM_POSITION)
                        .andThen(CoralArmCommands.goToPosition(arm, 1.1)));
    }

}
