// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    private Command AutonomousCommand;
    private RobotContainer RobotContainer;





    @Override
    public void robotInit()
    {
        RobotContainer = new RobotContainer();


        for (int port = 5800; port <= 5807; port++)
        {
            PortForwarder.add(port, "10.33.9.11", port);
        }

        RobotContainer.drivetrain.getDaqThread().setThreadPriority(99);

        System.out.println("Robot Initialized!");

        CommandScheduler.getInstance().schedule(Commands.sequence(
            Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink("")),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(""))
        ).ignoringDisable(true));

    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduledCOMPRESSOR
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        Vision.Periodic();

        // Returns apriltag data
        SmartDashboard.putNumber("Best Target Tx", LimelightHelpers.getTX(""));
        SmartDashboard.putNumber("Target ID #", LimelightHelpers.getFiducialID(""));
        SmartDashboard.putNumber("Target Ty", LimelightHelpers.getTY(""));
        SmartDashboard.putNumber("Target Ta", LimelightHelpers.getTA(""));

        // -- Output the robot orientation to the dashboard
        SmartDashboard.putNumber("Robot Yaw", RobotContainer.drivetrain.getPigeon2().getYaw().getValue());

        SmartDashboard.putNumber("Distance To Target", Vision.ReturnDistance());

    }


    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Disabled
    // -------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void disabledInit() { }

    @Override
    public void disabledPeriodic()
    {

    }


    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Auto
    // -------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void autonomousInit()
    {
        //AutonomousCommand = RobotContainer.GetAutonomousCommand();

        // schedule the autonomous command (example)
       // if (AutonomousCommand != null)
       // {
       //     AutonomousCommand.schedule();
      //  }
    }

    @Override
    public void autonomousPeriodic() { }


    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Teleop
    // -------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (AutonomousCommand != null)
        {
            AutonomousCommand.cancel();
        }

        RobotContainer.Arm.Command_SetNeutralMode(NeutralModeValue.Brake);
        RobotContainer.Intake.Command_SetNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void teleopPeriodic()
    {

    }


    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Test
    // -------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() { }


    // -------------------------------------------------------------------------------------------------------------------------------------
    // -- Simulation
    // -------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void simulationInit() { }

    @Override
    public void simulationPeriodic() { }
}
