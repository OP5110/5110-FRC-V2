// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.Commands.WaitSeconds;


//import edu.wpi.first.wpilibj2.command.Command.WaitCommand;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html



public final class BasicAuto {

  /** shooter auto */
  public static Command shootNote_pos3(SwerveSubsystem drivebase,ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    return Commands.sequence(

    armSubsystem.autoArmDown(),
    new WaitCommand(1),

    armSubsystem.stopArm(),

    shooterSubsystem.runShooter(-1),
    new WaitCommand(1.0),
    intakeSubsystem.runIntake(1),

    new WaitCommand(3),

    intakeSubsystem.stopIntake(),

    shooterSubsystem.stopShooter(),

    new WaitCommand(1),

    drivebase.getAutonomousCommand("POS3")
    );   
}

 public static Command Left_start(SwerveSubsystem drivebase,ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    return Commands.sequence(

    // lower arm
    armSubsystem.autoArmDown(),
    new WaitCommand(1),
    armSubsystem.stopArm(),

    // Shoot note
    shooterSubsystem.runShooter(-1),
    new WaitCommand(1.0),
    intakeSubsystem.runIntake(1),
    new WaitCommand(3),
    intakeSubsystem.stopIntake(),
    shooterSubsystem.stopShooter(),
    new WaitCommand(1),

    // goto to note
    drivebase.getAutonomousCommand("Left Out"),
    
// pickup note
    intakeSubsystem.runIntake(1),
    new WaitCommand(3),
    intakeSubsystem.stopIntake(),
    intakeSubsystem.runIntake(-.85),
    new WaitCommand(.5),
    intakeSubsystem.stopIntake(),

    drivebase.getAutonomousCommand("Left Return"),

    // shoot note
    shooterSubsystem.runShooter(-1),
    new WaitCommand(1.0),
    intakeSubsystem.runIntake(1),
    new WaitCommand(3),
    intakeSubsystem.stopIntake(),
    shooterSubsystem.stopShooter()

);
  }

  private BasicAuto() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}