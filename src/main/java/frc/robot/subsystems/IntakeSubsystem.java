package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

//import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_intake; 

    public IntakeSubsystem() {
        m_intake = new CANSparkMax(15, MotorType.kBrushless);

        m_intake.restoreFactoryDefaults();
        m_intake.setIdleMode(IdleMode.kCoast);
    }
   
 // Auto Commands
    public Command stopIntake(){
        return this.runOnce(() -> {
            m_intake.set(0);
        });
    }
    
    public Command runIntake(double speed){
        return this.runOnce(() -> {
            m_intake.set(speed);
        });
    }
   
    public Command autoRunIntake(){
        return startEnd(
            () -> m_intake.set(3),
            () -> m_intake.stopMotor()
        );
    }

    public Command autoRunIntakeRevers(){
        return startEnd(
            () -> m_intake.set(-.85),
            () -> m_intake.stopMotor()
        );
    }
    @Override
    public void periodic(){}
}