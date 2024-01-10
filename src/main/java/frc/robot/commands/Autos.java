// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos extends Command {

    public final Robot robot;
    public final SwerveSubsystem s_Swerve;
    public final DriveCommand d_Command;
    public final PneumaticsSubsystem p_sub;
    // private final SendableChooser<Command> autoChooser;



    public Autos(Robot robot, SwerveSubsystem s_Swerve, DriveCommand d_Command, PneumaticsSubsystem p_sub){
        this.robot = robot;
        this.s_Swerve= s_Swerve;
        this.d_Command = d_Command;
        this.p_sub = p_sub;

            
        // All my named commands
        NamedCommands.registerCommand("Open Intake", null);
        NamedCommands.registerCommand("Close Intake", null);
    }

    public Command testAuto(){
        return new PathPlannerAuto("TestAuto");
    }

    
    @Override
    public void initialize(){}
    @Override
    public void execute(){}
    @Override
    public void end(boolean interrupted) {}
    
    
    
    @Override
    public boolean isFinished() {
      return robot.autoCheck;
    }
    

    
  }
   
  
  // public Command testPath(){
  //     PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath");
    
  //     return new FollowPathWithEvents(
  //       new FollowPathHolonomic(
  //       path, 
  //       s_Swerve::getPose, 
  //       d_Command::getSpeeds, 
  //       s_Swerve::setModuleStates, 
  //       new HolonomicPathFollowerConfig(new PIDConstants(0.025, 0, 0), new PIDConstants(0.025, 0, 0), Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, Constants.ModuleConstants.moduleRadius, new ReplanningConfig(true, true)), s_Swerve), 
  //       path, 
  //       s_Swerve::getPose);
  //   }

// public void configureAutoCommands() {
//   // HashMap<String, Command> eventMap = new HashMap<>();
  
//           //Testing autos
//   // Command testAuto = AutoBuilder.buildAuto("TestAuto");
//   // PathPlannerAuto testAuto2 = new PathPlannerAuto("TestAuto");
//   // AutoBuilder.buildAuto("TestAuto");
//   // Command testAuto = new PathPlannerAuto("TestAuto");
  
//           //Testing paths
//   // Command testPath = AutoBuilder.followPathWithEvents(PathPlannerPath.fromPathFile("Test"));
//   // Command testPath = PathPlannerPath.fromPathFile("TestPath");
//   // FollowPathWithEvents eventsTest = new FollowPathWithEvents(testPath, PathPlannerPath.fromPathFile("Test"), s_Swerve::getPose);
  
//   SequentialCommandGroup Nothing = new SequentialCommandGroup(p_sub.extensionOutCommand());
//   autonChooser.addOption("Nothing", Nothing);
//   autonChooser.addOption("Test Path", testPath());
//   }
