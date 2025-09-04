package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arm extends SubsystemBase{
    private armIO armIO;
    public armIOInputsAutoLogged armInputs = new armIOInputsAutoLogged();

    public arm(armIO io){
        this.armIO = io;

        
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.processInputs("Arm/arm", armInputs);
    }

    public armIO getIO(){
        return armIO;
    }
}
