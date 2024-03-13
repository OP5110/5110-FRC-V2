// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BasicAuto;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystemL;
import frc.robot.subsystems.Climber.ClimberSubsystemR;

public class RobotContainer
{
  private final ClimberSubsystemL m_climberL = new ClimberSubsystemL();
  private final ClimberSubsystemR m_climberR = new ClimberSubsystemR();
  private final ArmSubsystem m_conv = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  XboxController driverXbox2 = new XboxController(1);
  XboxController driverXbox = new XboxController(0);

  //private final Command m_simpleAuto = Autos.simpleAuto(m_robotDrive);
  private final Command m_autoShootnote = BasicAuto.shootNote_pos3(drivebase, m_shooter, m_intake, m_conv);
private final Command m_autoLeft_start = BasicAuto.Left_start(drivebase, m_shooter, m_intake, m_conv);


  private void configureBindings()
  {
    // Climber, DPad
    //Constants.operatorController.povUp().onTrue(m_climber.setHeight(ClimberSubsystem.ClimberState.EXTENDED.height));
    //Constants.operatorController.povDown().onTrue(m_climber.setHeight(ClimberSubsystem.ClimberState.RETRACTED.height));
    
    Constants.operatorController.rightBumper().whileTrue(m_climberR.uhOhNoWorky(-.75)).whileFalse(m_climberR.uhOhNoWorkyStop());
    Constants.operatorController.rightTrigger().whileTrue(m_climberR.uhOhNoWorky(.75)).whileFalse(m_climberR.uhOhNoWorkyStop());
    
    Constants.operatorController.leftBumper().whileTrue(m_climberL.uhOhNoWorky2(-.75)).whileFalse(m_climberL.uhOhNoWorky2Stop());
    Constants.operatorController.leftTrigger(.1).whileTrue(m_climberL.uhOhNoWorky2(.75)).whileFalse(m_climberL.uhOhNoWorky2Stop());

    // Conv, Bumpers and run when inake
    Constants.operatorController.povUp().onTrue(m_conv.runArm(0.50))
      .onFalse(m_conv.runArm(0.03));
    Constants.operatorController.povDown().onTrue(m_conv.runArm(-.40))
      .onFalse(m_conv.runArm(0.03));
    
    Constants.operatorController.b().whileTrue(m_intake.autoRunIntake())
      .whileFalse(m_intake.stopIntake());
    Constants.operatorController.a().whileTrue(m_intake.autoRunIntakeRevers())
      .whileFalse(m_intake.stopIntake());
    
    // Shooter, LT & RT
    Constants.operatorController.x().whileTrue(m_shooter.runShooter(-1))
      .whileFalse(m_shooter.runShooter(0));

    // Default stuff remove eventually
    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    
   }

  
  /*public void configurePathPlanner() {
    // Conv
    NamedCommands.registerCommand("runConv", m_conv.autoRunArm());
    NamedCommands.registerCommand("stopConv", m_conv.stopArm());
    NamedCommands.registerCommand("autoArmDown", m_conv.autoArmDown());
    NamedCommands.registerCommand("stopArmUp", m_conv.autoArmUp());
    // Intake
    NamedCommands.registerCommand("runIntake", m_intake.autoRunIntake());
    NamedCommands.registerCommand("stopIntake", m_intake.stopIntake());
    // Shooter
    NamedCommands.registerCommand("runShooter", m_shooter.autoShooterRun());
    NamedCommands.registerCommand("stopShooter", m_shooter.stopShooter());

    drivebase.setupPathPlanner();
  }*/

   //private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    /*  Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser("Simple Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    configurePathPlanner();
    PortForwarder.add(5800, "photonvision.local", 5800);
*/
    configureBindings();
  }

  public void setDriveMode()
  {
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    drivebase.setDefaultCommand( !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngle );
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public Command getAutonomousCommand()
  {

return m_autoLeft_start;

//   return drivebase.getAutonomousCommand("POS3");

 }

}
