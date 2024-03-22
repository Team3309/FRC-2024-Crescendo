// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import friarLib2.math.LookupTable;



public class RobotContainer
{
    static private RobotContainer _This;
    static public RobotContainer Get() { return _This; }
    static public final PowerDistribution PDH = new PowerDistribution();

    // --------------------------------------------------------------------------------------------
    // -- Controllers
    // --------------------------------------------------------------------------------------------
    static public CommandXboxController Driver = new CommandXboxController(0);
    static public CommandXboxController Operator = new CommandXboxController(1);


    // --------------------------------------------------------------------------------------------
    // -- Misc Input
    // --------------------------------------------------------------------------------------------
    static private final DigitalInput DIO_Zero = new DigitalInput(9);
    static private final Trigger Trigger_Zero = new Trigger(() -> DIO_Zero.get());


    // --------------------------------------------------------------------------------------------
    // -- Tuning Values
    // --------------------------------------------------------------------------------------------
    private final double MaxRotationsPerSecond = 0.75;
    private LookupTable ThrottleLut = new LookupTable.Normalized()
        .AddValue(0.1, 0.0) // dead-band
        .AddValue(0.35, 0.05)
        .AddValue(0.75, 0.2);

    private LookupTable TurnLut = new LookupTable.Normalized()
        .AddValue(0.1, 0.0) // dead-band
        .AddValue(0.35, 0.05)
        .AddValue(0.75, 0.2);

    private final DriverStation.Alliance DefaultAlliance = DriverStation.Alliance.Blue;

    private DriverStation.Alliance CurrentAlliance = DriverStation.getAlliance().get();

    // --------------------------------------------------------------------------------------------
    // -- Subsystems
    // --------------------------------------------------------------------------------------------
    public final SwerveSubsystem drivetrain = TunerConstants.DriveTrain;
    public final ArmSubsystem Arm = new ArmSubsystem();
    public final IntakeSubsystem Intake = new IntakeSubsystem();
    public final PoseManager Pose = new PoseManager(Arm, Intake);
    
    
    
    // --------------------------------------------------------------------------------------------
    // -- Drive requests
    // --------------------------------------------------------------------------------------------
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle driveTowardsAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final SwerveRequest.RobotCentric AimRobot = new SwerveRequest.RobotCentric().withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);



    // --------------------------------------------------------------------------------------------
    // -- Auto Chooser
    // --------------------------------------------------------------------------------------------
    private final SendableChooser<Command> autoChooser;

    
    
    // --------------------------------------------------------------------------------------------
    // -- State
    // --------------------------------------------------------------------------------------------
    private boolean RotationModeIsRobotCentric = false;
    
    

    public RobotContainer()
    {
        _This = this;



        ConfigureAutoCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.addOption("None", Commands.none());
        autoChooser.addOption("SimpleForward", Commands.run(() -> driveFieldCentric.withVelocityX(1)).withTimeout(2));
        autoChooser.addOption("Score Preload", Commands.sequence(
            Pose.Command_GoToPose(PoseManager.EPose.Speaker),
            Commands.waitSeconds(0.5),
            Intake.Command_Outtake(IntakeSubsystem.EOuttakeType.speaker),
            Pose.Command_GoToPose(PoseManager.EPose.Stowed)));

        SetDefaultCommands();

        ConfigureMiscBindings();
        ConfigureDriverBindings();
        ConfigureOperatorBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
   }
   
    Command Command_AlignToTag()
    {
        return drivetrain.applyRequest(this::GetAlignToTagRequest);
    }
    
   public Command Command_RumbleControllers()
   {
       return Commands.runOnce(() ->
           CommandScheduler.getInstance().schedule(
               Commands.sequence(
                   Commands.waitSeconds(0.5),
                   Commands.runOnce(() -> {
                       Operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
                       Driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
                   }),
                   Commands.waitSeconds(0.5),
                   Commands.runOnce(() -> {
                       Operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                       Driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                   })
               )
           )
       );
   }

   Command Command_IntakeNoteSequence(boolean fromSource)
    {
        return Commands.sequence(
            Arm.Command_SetPosition(fromSource ? ArmSubsystem.EArmPosition.Source : ArmSubsystem.EArmPosition.Stowed),
            Intake.Command_IntakeNote(fromSource)
        );
    }

    public Command Command_AutoPose()
    {
        return Commands.runOnce(() -> Pose.Command_GoToPose(GetPoseForTag()));

    }

    //    {return Commands.runOnce(() -> Pose.Command_GoToPose(Pose.GetPoseForCurrentTag()));}


    private void ConfigureAutoCommands()
    {
        NamedCommands.registerCommand("Arm Score", Pose.Command_GoToPose(PoseManager.EPose.Speaker));
        NamedCommands.registerCommand("Arm Stow", Pose.Command_GoToPose(PoseManager.EPose.Stowed));
        NamedCommands.registerCommand("Intake Note",
            Command_IntakeNoteSequence(false)
            .withTimeout(2.5)
            .andThen(Pose.Command_GoToPose(PoseManager.EPose.Stowed))
            .andThen(Intake.Command_StopIntake()));
        NamedCommands.registerCommand("Stop Intake motors", Intake.Command_StopIntake());
        NamedCommands.registerCommand("Condition Stop Intake", Intake.Command_ConditionalStowAuto());
        NamedCommands.registerCommand("Shoot Speaker", Intake.Command_Outtake(IntakeSubsystem.EOuttakeType.speaker).withTimeout(0.5).andThen(Intake.Command_StopIntake()));
    }
    
    public Command GetAutonomousCommand()
    {
        return autoChooser.getSelected();
    }

    
    
    private void SetDefaultCommands()
    {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(this::GetDefaultDriveRequest).ignoringDisable(true));
    }

    private void ConfigureMiscBindings()
    {
        Trigger_Zero.onTrue(
            Commands.sequence(
                Intake.Command_ZeroPivotEncoder(),
                Arm.Command_ZeroArmEncoder(),
                drivetrain.runOnce(drivetrain::seedFieldRelative),
                Commands.print("Zeroed!"),
                Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink("")),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(""))
            )
            .unless(() -> DriverStation.isEnabled())
            .ignoringDisable(true));
    }

    // --------------------------------------------------------------------------------------------
    // -- Driver
    // --------------------------------------------------------------------------------------------
    private void ConfigureDriverBindings()
    {

        // Slow drive relative to bot pose
        Driver.povUp().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(.5))); // Fine-tune control forwards
        Driver.povDown().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(-.5))); // Fine-tune control backwards
        Driver.povRight().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityY(-.5))); // Fine-tune control right
        Driver.povLeft().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityY(.5))); // Fine-tune control left

        Driver.povUpRight().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(.5).withVelocityY(-.5))); // Fine-tune control diagonally up and right
        Driver.povUpLeft().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(.5).withVelocityY(.5))); // Fine-tune control diagonally up and left
        Driver.povDownLeft().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(-.5).withVelocityY(.5))); // Fine-tune control diagonally down and left
        Driver.povDownRight().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(-.5).withVelocityY(-.5))); // Fine-tune control diagonally down and right

        //Driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //Driver.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))));

        Driver.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative).ignoringDisable(true));

        // -- Intake
        Driver.rightTrigger().onTrue(Command_IntakeNoteSequence(false));
        Driver.rightTrigger().onFalse(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> Intake.RequestCancelIntake()))
        );
        Driver.rightTrigger().onFalse(Commands.sequence(
            Command_AutoPose(),
            Commands.waitSeconds(1),
            Pose.Command_GoToPose(PoseManager.EPose.Stowed)
        ));

        // -- Outtake speaker
        Driver.leftTrigger().onTrue(
            Commands.sequence(
                //Pose.Command_GoToPose(PoseManager.EPose.Speaker),
                Intake.Command_Outtake(IntakeSubsystem.EOuttakeType.speaker),
                Pose.Command_GoToPose(PoseManager.EPose.Stowed))
        );

        // -- Outtake Amp
        Driver.leftBumper().onTrue(
            Commands.sequence(
                Intake.Command_Outtake(IntakeSubsystem.EOuttakeType.amp),
                Pose.Command_GoToPose(PoseManager.EPose.Stowed))
        );

        // -- Align
        Driver.rightBumper().whileTrue(Command_AlignToTag());

//        Driver.rightBumper().onFalse(Commands.sequence(
//            Intake.Command_Outtake(IntakeSubsystem.EOuttakeType.amp)
//        ));

//        Driver.rightBumper().onFalse(Commands.sequence(
//            Commands.waitSeconds(1),
//            Command_AutoPose()
//        ));

        // -- Testing for autoPosing and Outtaking depending on apriltag
        Driver.a().onTrue(Command_AutoPose());
        Driver.b().onTrue(Intake.Command_Outtake(GetOuttakeType()));

    }
    
    
    
    // --------------------------------------------------------------------------------------------
    // -- Operator
    // --------------------------------------------------------------------------------------------
    private void ConfigureOperatorBindings()
    {
        Operator.start().onTrue(Arm.Command_ZeroArmEncoder().alongWith(Intake.Command_ZeroPivotEncoder().ignoringDisable(true)));
        Operator.back().whileTrue(Arm.Command_ManualArmControl());



        Operator.a().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Stowed));
        Operator.b().onTrue(Intake.Command_UnstickPivot());
        Operator.x().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Amp));
        Operator.y().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Speaker));

        Operator.povUp().onTrue(Pose.Command_GoToPose(PoseManager.EPose.PreClimb));
        Operator.povDown().onTrue(Arm.Command_Climb());

        Operator.povLeft().onTrue(Arm.Command_SetNeutralMode(NeutralModeValue.Brake).alongWith(Intake.Command_SetNeutralMode(NeutralModeValue.Brake)));
        Operator.povRight().onTrue(Arm.Command_SetNeutralMode(NeutralModeValue.Coast).alongWith(Intake.Command_SetNeutralMode(NeutralModeValue.Coast)));

        Operator.rightBumper().whileTrue(Intake.Command_MoveNote(false));
        Operator.leftBumper().whileTrue(Intake.Command_MoveNote(true));

        Operator.leftTrigger().onTrue(Intake.Command_FeederTakeNote(false));

        Operator.rightTrigger().onTrue(Command_IntakeNoteSequence(true));
        Operator.rightTrigger().onFalse(Pose.Command_GoToPose(PoseManager.EPose.Stowed).andThen(Commands.runOnce(() -> Intake.RequestCancelIntake())));
    }

    public SwerveRequest GetDefaultDriveRequest()
    {
        double y = Driver.getLeftX();
        double x = Driver.getLeftY();

        double deflection = Math.sqrt(x*x + y*y);
        double deflectionLut = ThrottleLut.GetValue(deflection);
        if (deflectionLut == 0)
        {
            return driveFieldCentric
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(GetDefaultDriveRotationRate());
        }

        double finalX = x * deflectionLut * TunerConstants.kSpeedAt12VoltsMps;
        double finalY = y * deflectionLut * TunerConstants.kSpeedAt12VoltsMps;

        if (RotationModeIsRobotCentric)
        {
            return driveRobotCentric
                .withVelocityX(-finalX)
                .withVelocityY(-finalY)
                .withRotationalRate(GetDefaultDriveRotationRate());
        }
        else
        {
            return driveFieldCentric
                .withVelocityX(-finalX)
                .withVelocityY(-finalY)
                .withRotationalRate(GetDefaultDriveRotationRate());
        }
    }

    private double GetDefaultDriveRotationRate()
    {
        double magicScalar = 0.02; // The steering gains seem borked, but I ran out of time to figure out how to tune them correctly, so here's a magic number to get it to turn decently.

        double finalAngVelocity = -TurnLut.GetValue(Driver.getRightX()) * MaxRotationsPerSecond * 360;
        return finalAngVelocity * magicScalar;
    }

    private SwerveRequest GetAlignToTagRequest()
    {
        var target = Vision.GetBestTarget();
        if (target == null)
        {
            return GetDefaultDriveRequest();
        }

        Pose3d pose = target.getRobotPose_TargetSpace();
        double posY = pose.getTranslation().getZ();
        double posX = pose.getTranslation().getX();
        double angleY = pose.getRotation().getY();

        double rotationalOffset = 0.05;

        SmartDashboard.putNumber("Target.posX", posX);
        SmartDashboard.putNumber("Target.posY", posY);
        SmartDashboard.putNumber("Target.angleY", angleY);

        return driveRobotCentric
            .withVelocityX(-(posY + 1))
            .withVelocityY(posX * 2)
            .withRotationalRate((angleY + rotationalOffset) * 7);
    }

    public PoseManager.EPose GetPoseForTag()
    {
        var target = Vision.GetBestTarget();

        var id = target.fiducialID;

        if (target == null)
        {
            System.out.println("No target");
            return PoseManager.EPose.None; }

        if (id == 4 || id == 7)
        {
            System.out.println("SpeakerPose");
            return PoseManager.EPose.Speaker;
        }
        else if (id == 5 || id == 6)
        {
            System.out.println("AmpPose");
            return PoseManager.EPose.Amp;
        }
        else if (id >= 11 || id <= 16)
        {
            System.out.println("Climb");
            return PoseManager.EPose.PreClimb;
        }
        System.out.println("No Pose");
        return PoseManager.EPose.None;
    }

    public IntakeSubsystem.EOuttakeType GetOuttakeType()
    {
        var target = Vision.GetBestTarget();
        var id = target.fiducialID;

        if (target == null)
        {
            System.out.println("No target");
            return IntakeSubsystem.EOuttakeType.None;
        }

        if (id == 5 || id == 6)
        {
            System.out.println("Amp Outtake");
            return IntakeSubsystem.EOuttakeType.amp;
        }
        else if (id == 4 || id == 7)
        {
            System.out.println("Speaker Outtake");
            return IntakeSubsystem.EOuttakeType.speaker;
        }

        System.out.println("No outtake type");
        return IntakeSubsystem.EOuttakeType.None;

    }

    
}
