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
        // return ElevatorCommands.goToPosition(elevator,
        // L2_ELEVATOR_POSITION)
        // .until(() -> elevator.getIO().getPosition() - 0.4 >= L1_ELEVATOR_POSITION)
        // .andThen(CoralArmCommands.goToPosition(arm, L1_ARM_POSITION));

        return ElevatorCommands.goToPosition(elevator, 16.7);
    }

    public Command L2(Elevator elevator, CoralArm arm) {
        return ElevatorCommands.goToPosition(elevator,
                L4_ELEVATOR_POSITION)
                .until(() -> elevator.getIO().getPosition() - 0.4 >= L2_ELEVATOR_POSITION)
                .andThen(CoralArmCommands.goToPosition(arm, L2_ARM_POSITION));
    }

    public Command L3(Elevator elevator, CoralArm arm) {
        return CoralArmCommands.goToPosition(arm, 1.1);
    }

    public Command L4(Elevator elevator, CoralArm arm) {
        return ElevatorCommands.goToPosition(elevator, L4_ELEVATOR_POSITION)
                .until(() -> elevator.getIO().getPosition() - 0.4 >= L4_ELEVATOR_POSITION)
                .alongWith(CoralArmCommands.goToPosition(arm, L4_ARM_POSITION)
                        .andThen(() -> arm.getIO().setVoltage(0.3)));
    }

    // L4
    // operatorController.y().onTrue(CoralArmCommands.goToPosition(coralArm,
    // L4_ARM_POSITION)
    // .until(() -> coralArm.getIO().getPosition() >= L4_ARM_POSITION)
    // .andThen(ElevatorCommands.goToPosition(elevator, L4_ELEVATOR_POSITION)));

    // operatorController.y()
    // .onTrue(ElevatorCommands.goToPosition(elevator, 33.5)
    // .until(() -> elevator.getIO().getPosition()
    // - 0.4 >= L4_ELEVATOR_POSITION)
    // .andThen(CoralArmCommands.goToPosition(coralArm,
    // L4_ARM_POSITION)));

    // L3
    // operatorController.x().onTrue(CoralArmCommands.goToPosition(coralArm,
    // L3_ELEVATOR_POSITION));

    // L2
    // operatorController.a().onTrue(ElevatorCommands.goToPosition(elevator,
    // L4_ELEVATOR_POSITION)
    // .until(() -> elevator.getIO().getPosition() - 0.4 >= L2_ELEVATOR_POSITION)
    // .andThen(CoralArmCommands.goToPosition(coralArm, L2_ARM_POSITION)));

    // L1
    // operatorController.LB().onTrue(ElevatorCommands.goToPosition(elevator,
    // L2_ELEVATOR_POSITION)
    // .until(() -> elevator.getIO().getPosition() - 0.4 >= L1_ELEVATOR_POSITION)
    // .andThen(CoralArmCommands.goToPosition(coralArm, L1_ARM_POSITION)));
}
