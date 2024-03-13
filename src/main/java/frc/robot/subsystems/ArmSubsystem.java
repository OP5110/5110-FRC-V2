package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax armMotor;
    private RelativeEncoder arm_encoder;
    
    public ArmSubsystem() {
        armMotor = new CANSparkMax(9, MotorType.kBrushless);
        armMotor.setSmartCurrentLimit(10);
        armMotor.restoreFactoryDefaults();
        armMotor.setIdleMode(IdleMode.kBrake);
        arm_encoder = armMotor.getEncoder();
    }
    
    public Command runArm(double speed){
        return startEnd(
            () -> armMotor.set(speed),
            () -> armMotor.stopMotor()
        );
    }
    public Command runArmIntake(){
        return runArm(0.5);
    }

    // Auto
    public Command autoRunArm(){
        return runArm(-0.3);
    }

    public Command autoArmUp(){
        return this.runOnce(
            () -> armMotor.set(.25)
        );
    }

    public Command autoArmDown(){
        return this.runOnce(
            () -> armMotor.set(-.2)
        );
    }


    public Command stopArm(){
        return this.runOnce(
            () -> armMotor.stopMotor()
        );
    }
}
