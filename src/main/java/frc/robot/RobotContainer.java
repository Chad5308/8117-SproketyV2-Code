// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShootingCommand;
import frc.robot.commands.TransferCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drivebase.LimelightSubsystem;
import frc.robot.subsystems.Drivebase.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;



/**
 * This class is where the bulk
 *  of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);
  private final CommandXboxController shootController = new CommandXboxController(Constants.OIConstants.kShooterControllerPort);
  // private CommandJoystick driveStick = new CommandJoystick(0);
  public static Robot robot = new Robot();
  public ShooterSubsystem shooter_sub = new ShooterSubsystem();
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public PitchSubsystem pitchSubsystem = new PitchSubsystem();
  public SwerveSubsystem s_Swerve = new SwerveSubsystem(robot, shooter_sub, intakeSubsystem);
  public LimelightSubsystem LL_sub = new LimelightSubsystem(s_Swerve,pitchSubsystem);
  public ShootingCommand shootingCommand = new ShootingCommand(shooter_sub, pitchSubsystem, LL_sub);
  public DriveCommand d_Command = new DriveCommand(s_Swerve, LL_sub, opController);
  public TransferCommand transferCommand = new TransferCommand(shooter_sub, intakeSubsystem, pitchSubsystem);
  public AutoCommand autoCommand = new AutoCommand(shootingCommand, transferCommand, d_Command, s_Swerve, intakeSubsystem, shooter_sub, pitchSubsystem);
  private SendableChooser<Command> autoChooser;


 
public RobotContainer() {
  s_Swerve.setDefaultCommand(new DriveCommand(s_Swerve, LL_sub, opController));
  configureBindings();

  shooter_sub.setDefaultCommand(shootingCommand);
  pitchSubsystem.setDefaultCommand(shootingCommand);



      
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);    


  }

  public void configureAutos(){
    autoChooser.addOption("Close Speaker Stay", shootBreachStay());
    autoChooser.addOption("1", one());
    autoChooser.addOption("2", two());
    autoChooser.addOption("SourceShootLeave", SourceShootLeave());
    autoChooser.addOption("AmpScoreLeave", AmpScoreLeave());
  }

  public Command shootBreachStay(){
    return Commands.runOnce(() -> {shootingCommand.closeSpeaker();});}
  public Command one(){
    return new PathPlannerAuto("1");}
  public Command two(){
    return new PathPlannerAuto("2");}
  public Command SourceShootLeave(){
    return new PathPlannerAuto("SourceShootLeave");}
  public Command AmpScoreLeave(){
    return new PathPlannerAuto("AmpScoreLeave");
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  
  private void configureBindings() {
    //Drive Controls
    opController.povRight().toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    opController.povLeft().toggleOnTrue(s_Swerve.fieldOrientedToggle());
    opController.button(7).onTrue(s_Swerve.resetWheels()); //window looking button

    opController.axisGreaterThan(3, 0.25).whileTrue(intakeSubsystem.runIntakeCommand());
    opController.axisGreaterThan(3, 0.25).whileFalse(intakeSubsystem.stopIntakeCommand());
    opController.axisGreaterThan(2, 0.25).whileTrue(intakeSubsystem.dropIntakeCommand());
    opController.axisGreaterThan(2, 0.25).whileFalse(intakeSubsystem.liftIntakeCommand());

    opController.rightBumper().onTrue(Commands.runOnce(() -> {d_Command.autoAlign = !d_Command.autoAlign;}));
    //shooter Controls
    // shootController.a().onTrue(shooter_sub.slowShooter());
    // shootController.y().onTrue(shooter_sub.fastShooter());
    shootController.axisGreaterThan(3, 0.25).whileTrue(Commands.runOnce(() -> {shootingCommand.shootSpeedBreach();}));
    shootController.axisGreaterThan(2, 0.25).whileTrue(Commands.runOnce(() -> {shootingCommand.reverseBreach();}));
    shootController.axisGreaterThan(3, 0.25).whileFalse(Commands.runOnce(()-> {shootingCommand.stopBreach();}));
    shootController.axisGreaterThan(2, 0.25).whileFalse(Commands.runOnce(()-> {shootingCommand.stopBreach();}));
    
    
    shootController.povLeft().onTrue(Commands.runOnce(()->{shootingCommand.speedUpFlyWheels();}));
    shootController.povRight().onTrue(Commands.runOnce(()->{shootingCommand.stopFlyWheels();}));
    shootController.povUp().whileTrue(Commands.run(()->{pitchSubsystem.rotatePositive();}));
    shootController.povDown().whileTrue(Commands.run(()->{pitchSubsystem.rotateNegative();}));

    //
    
    //test for PID
    // shootController.y().onTrue(Commands.runOnce(() -> {pitchSubsystem.setPosition(Constants.ShooterConstants.closeSpeakerAngle);}));
    
    
    
    
    
    //Auto fire Controls
    shootController.a().onTrue(Commands.runOnce(() -> {shootingCommand.closeSpeaker();}));
    shootController.b().onTrue(Commands.runOnce(()-> {pitchSubsystem.setPosition(Constants.ShooterConstants.sourceAngle);}));
    shootController.y().onTrue(Commands.runOnce(() -> {shootingCommand.podiumShot();}));

    shootController.rightBumper().onTrue(Commands.runOnce(() -> {shootingCommand.isFlipped = !shootingCommand.isFlipped;}));
    shootController.leftBumper().onTrue(Commands.runOnce(() -> {shootingCommand.autoAim = !shootingCommand.autoAim;}));
    
    // shootController.rightBumper().whileTrue(Commands.runOnce(()->{shootingCommand.autoAim();}));


  }
}
