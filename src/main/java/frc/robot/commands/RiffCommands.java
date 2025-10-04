package frc.robot.commands;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.subsystems.CoralArm.CoralArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.CoralArm.CoralArm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Transfer.Transfer;

public class RiffCommands {
    Elevator elevator;
    CoralArm arm;
    Transfer transfer;

    public RiffCommands(Elevator elevator, CoralArm arm, Transfer transfer) {
        this.elevator = elevator;
        this.arm = arm;
        this.transfer = transfer;
    }

    public Command L1() {
        return ElevatorCommands.goToPosition(elevator, L1_ELEVATOR_POSITION)
                .alongWith(CoralArmCommands.closeArm(arm))
                .withName("L1");
    }

    public Command L2() {
        return ElevatorCommands.goToPosition(elevator,
                L4_ELEVATOR_POSITION)
                .until(() -> elevator.getIO().getPosition() - 0.4 >= L2_ELEVATOR_POSITION)
                .andThen(CoralArmCommands.goToPosition(arm, L2_ARM_POSITION))
                .withName("L2");
    }

    // public Command L3() {
    // return Commands.sequence(
    // Commands.parallel(
    // // CoralArmCommands.goToPosition(arm, L4_ARM_POSITION),
    // ElevatorCommands.closeElevator(elevator)),
    // Commands.parallel(
    // ElevatorCommands.stopElevator(elevator),
    // CoralArmCommands.goToPosition(arm, 1.1)))
    // .withName("L3");
    // // return CoralArmCommands.goToPosition(arm, L4_ARM_POSITION)
    // // .andThen(ElevatorCommands.closeElevator(elevator)
    // // .andThen(ElevatorCommands.stopElevator(elevator))
    // // .andThen(CoralArmCommands.goToPosition(arm, 1.1)))
    // // .withName("L3");

    // }

    public Command L3() {
        return new ConditionalCommand(
                CoralArmCommands.goToPosition(arm, 1.1), Commands.parallel(
                        CoralArmCommands.goToPosition(arm, OPEN_ARM_POSITION),
                        ElevatorCommands.goToPosition(elevator, CLOSE_ELEVATOR_POSITIOM)),
                arm.getIO()::getHighSwitch).withName("L3");
    }

    // public Command L4() {
    // return Commands.sequence(
    // Commands.parallel(
    // ElevatorCommands.goToPosition(elevator, L4_ELEVATOR_POSITION),
    // CoralArmCommands.goToPosition(arm, L),
    // CoralArmCommands.goToPosition(arm, 1.1))).withName("L4");

    // // return ElevatorCommands.goToPosition(elevator, L4_ELEVATOR_POSITION)
    // // .until(() -> elevator.getIO().getPosition() - 0.4 >= L4_ELEVATOR_POSITION)
    // // .alongWith(CoralArmCommands.goToPosition(arm, L4_ARM_POSITION)
    // // .andThen(CoralArmCommands.goToPosition(arm, 1.1)))
    // // .withName("L4");
    // }

    public Command L4() {
        return new ConditionalCommand(
                CoralArmCommands.goToPosition(arm, 1.1), (Commands.parallel(
                        CoralArmCommands.goToPosition(arm, 1.6).until(arm.getIO()::getHighSwitch),
                        ElevatorCommands.goToPosition(elevator, 34)
                                .until(() -> elevator.getIO().getPosition() > 33.1))),
                arm.getIO()::getHighSwitch).withName("L4");
    }

    public Command coralIntakePos() {
        return (ElevatorCommands.goToPosition(elevator, 2.1)
                .alongWith(CoralArmCommands.closeArm(arm))
                .andThen(TransferCommands.autoIntakeCoral(transfer)))
                .unless(transfer.getIO()::isCoralIn)
                .andThen(ElevatorCommands.goToPosition(elevator, 15.0))
                .withName("coralIntakePos");
    }

    public Command Lx(int x) {
        switch (x) {
            case 1:
                return L1();
            case 2:
                return L2();
            case 3:
                return L3();
            case 4:
                return L4();
            default:
                return Commands.print("Invalid L");
        }
    }

    public Command closeAll() {
        return CoralArmCommands.closeArm(arm)
                .alongWith(ElevatorCommands.closeElevator(elevator)
                        .andThen(ElevatorCommands.closeElevator(elevator)))
                .withName("closeAll");
    }

}
