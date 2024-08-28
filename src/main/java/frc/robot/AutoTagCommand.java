package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import friarLib2.math.FriarMath;


public class AutoTagCommand extends Command
{
    LimelightHelpers.LimelightTarget_Fiducial CurrentTarget;

    PIDController AimPID = new PIDController(0.165, 0, 0.01);
    PIDController XPID = new PIDController(2, 0, 0);
    PIDController YPID = new PIDController(2, 0, 0);

    boolean IsShooting;

    boolean HasNote;


    public AutoTagCommand()
    {
        AimPID.setTolerance(5);
        YPID.setTolerance(.025);
        XPID.setTolerance(.08);
    }

    @Override
    public void initialize()
    {
        IsShooting = false;
        HasNote = false;
    }

    @Override
    public void execute()
    {
        var name = "";

        var target = Vision.GetBestTarget();
        if (target == null)
        {
            var request = RobotContainer.Get().GetDefaultDriveRequest();
            RobotContainer.Get().drivetrain.setControl(request);
            return;
        }

        CurrentTarget = target;

        int id = (int)target.fiducialID;

        if (id == 4 || id == 7)
        {
            ExecuteSpeaker();
            return;
        }

        if (id == 5 || id ==6)
        {
            ExecuteAmp();
            return;
        }

//        if (id == 1 || id == 2 || id == 9 || id == 10)
        if (id == 1 || id == 9)
        {
            ExecuteSource();
            return;
        }

        if (id >= 11)
        {
            ExecuteStage();
        }
    }

    private void ExecuteSpeaker()
    {
        double minDist = 1.52;
        //double crossOver = 2.3;
        double crossOver = 2.45;
        double maxDist = 3; // Might have to change

        double minArmRot = -0.058; // OG: -.07
        double maxArmRot = ArmSubsystem.EArmPosition.Stowed.Rotations + 0.01;

        double minPivRot = IntakeSubsystem.EPivotPosition.Stowed.Rotations;
        //double maxPivRot = -0.286; //OG
        double maxPivRot = -0.26;


        // -- Auto Moving Arm
        Pose3d pose = CurrentTarget.getRobotPose_TargetSpace();
        var x = pose.getTranslation().getZ();
        var y = pose.getTranslation().getX();
        var dist = Math.sqrt((x * x) + (y * y));
        dist = MathUtil.clamp(dist, minDist, maxDist);
        SmartDashboard.putNumber("AutoTag.speaker.dist", dist);
        SmartDashboard.putNumber("AutoTag.speaker.y", y);

        var armPos = minArmRot;
        var pivotPos = minPivRot;

        if (dist < crossOver)
        {
            armPos = FriarMath.Remap(dist, minDist, crossOver, minArmRot, maxArmRot, false);
        }
        else
        {
            pivotPos = FriarMath.Remap(dist, crossOver, maxDist, minPivRot, maxPivRot, false);
        }
        SmartDashboard.putNumber("AutoTag.speaker.arm", armPos);
        SmartDashboard.putNumber("AutoTag.speaker.pivot", pivotPos);
       // RobotContainer.Get().Arm.SetArmPosRaw(aimAngle);

        // -- Auto Lineup
        var targetAngle = Math.toDegrees(Math.asin(-x / dist)) - 90;
        if ( y < 0)
        {
            targetAngle *= -1;
        }
        var currentAngle = Math.toDegrees(pose.getRotation().getY());

        SmartDashboard.putNumber("AutoTag.speaker.theta", targetAngle);
        SmartDashboard.putNumber("AutoTag.speaker.robotAngle", currentAngle);

        var rotationRate = AimPID.calculate(currentAngle, targetAngle);


        //var request = RobotContainer.Get().GetDefaultDriveRequest();
        var output = RobotContainer.Get().GetVelocityForThrottle();
        var request = RobotContainer.Get().driveFieldCentric
            .withVelocityX(output.getX())
            .withVelocityY(output.getY())
            .withRotationalRate(-rotationRate);
        RobotContainer.Get().drivetrain.setControl(request);


        if (dist <= maxDist && !IsShooting && AimPID.atSetpoint())
        {
            System.out.println("At setpoint");
            if (dist < crossOver)
            {
                System.out.println("Moving Arm");
                CommandScheduler.getInstance().schedule(
                    RobotContainer.Get().Arm.Command_GoToPosition(armPos)
                        .andThen(Commands.waitSeconds(.5))
                    .andThen(RobotContainer.Get().Command_ScoreSpeaker()));
            }
            else if (dist == crossOver)
            {
                System.out.println("Shooting");
                CommandScheduler.getInstance().schedule(
                    RobotContainer.Get().Command_ScoreSpeaker()
                );
            }
            else
            {
                System.out.println("Moving Pivot");
                CommandScheduler.getInstance().schedule(
                    RobotContainer.Get().Intake.Command_GoToPivotPosition(pivotPos)
                        .andThen(Commands.waitSeconds(.5))
                    .andThen(RobotContainer.Get().Command_ScoreSpeaker()));
            }
            IsShooting = true;
        }

    }

    private void ExecuteAmp()
    {
        Pose3d pose = CurrentTarget.getRobotPose_TargetSpace();
        double posY = pose.getTranslation().getX();
        double posX = pose.getTranslation().getZ();
        double angleY = Math.toDegrees(pose.getRotation().getY());
        SmartDashboard.putNumber("AutoTag.posX", posX);
        SmartDashboard.putNumber("AutoTag.posY", posY);
        SmartDashboard.putNumber("AutoTag.angleY", angleY);

        //double rotationalOffset = 0.05;
        var rotationRate = AimPID.calculate(angleY, 0);
        SmartDashboard.putNumber("AutoTag.rotationRate", rotationRate);

        double offset = 0.05;

        double finalX = XPID.calculate(posX, -1.1);
        double finalY = YPID.calculate(posY, 0 + offset);

        var request = RobotContainer.Get().driveRobotCentric
        .withVelocityX(finalX)
        .withVelocityY(-finalY)
        .withRotationalRate(-rotationRate);

        RobotContainer.Get().drivetrain.setControl(request);

        if (!IsShooting && XPID.atSetpoint() && YPID.atSetpoint() && AimPID.atSetpoint())
        {
            CommandScheduler.getInstance()
                            .schedule(Commands.sequence(
                                RobotContainer.Get().Pose.Command_GoToPose(PoseManager.EPose.Amp),
                                RobotContainer.Get().Command_DriveForward(
                                    1, .45),
                                Commands.waitSeconds(.5),
                                RobotContainer.Get().Command_ScoreAmp()
                            ));

            IsShooting = true;
        }

    }

    private void ExecuteSource()
    {
        Pose3d pose = CurrentTarget.getRobotPose_TargetSpace();
        var posX = pose.getTranslation().getZ();
        var posY = pose.getTranslation().getX();
        var angleY = Math.toDegrees(pose.getRotation().getY());
        SmartDashboard.putNumber("AutoTag.posX", posX);
        SmartDashboard.putNumber("AutoTag.posY", posY);
        SmartDashboard.putNumber("AutoTag.angleY", angleY);

        var rotationRate = AimPID.calculate(angleY, 0);
        SmartDashboard.putNumber("AutoTag.rotationRate", rotationRate);

        double finalX = XPID.calculate(posX, -1.1);
        double finalY = YPID.calculate(posY, 0);

        var request = RobotContainer.Get().driveRobotCentric
            .withVelocityX(finalX)
            .withVelocityY(-finalY)
            .withRotationalRate(-rotationRate);

        RobotContainer.Get().drivetrain.setControl(request);

//        if (!IsShooting && !HasNote && XPID.atSetpoint() && YPID.atSetpoint())
//        {
//            CommandScheduler.getInstance().schedule(Commands.sequence(
//                RobotContainer.Get().Command_DriveForward(-1, 1),
//                Commands.waitSeconds(.5),
//                RobotContainer.Get().Pose.Command_GoToPose(PoseManager.EPose.Source),
//                RobotContainer.Get().Intake.Command_IntakeNote(true),
//                RobotContainer.Get().Pose.Command_GoToPose(PoseManager.EPose.Stowed))
//            );
//            IsShooting = true;
//        }
    }

    private void ExecuteStage()
    {
        Pose3d pose = CurrentTarget.getRobotPose_TargetSpace();
        var posX = pose.getTranslation().getZ();
        var posY = pose.getTranslation().getX();
        var angleY = Math.toDegrees(pose.getRotation().getY());
        SmartDashboard.putNumber("AutoTag.posX", posX);
        SmartDashboard.putNumber("AutoTag.posY", posY);
        SmartDashboard.putNumber("AutoTag.angleY", angleY);

        var rotationRate = AimPID.calculate(angleY, 0);

        double offset = 0.05;

        double finalX = XPID.calculate(posX, -1);
        double finalY = YPID.calculate(posY, 0 + offset);

        var request = RobotContainer.Get().driveRobotCentric
            .withVelocityX(finalX)
            .withVelocityY(-finalY)
            .withRotationalRate(-rotationRate);

        RobotContainer.Get().drivetrain.setControl(request);

    }

}
