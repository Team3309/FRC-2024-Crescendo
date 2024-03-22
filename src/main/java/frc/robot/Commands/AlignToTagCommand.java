package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Vision;

public class AlignToTagCommand extends Command
{    
    final boolean FinishWhenClose;
    final SwerveDrivetrain Drivetrain;
    final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    
    
    final PIDController PIDDistance = new PIDController(0, 0, 0); // Pose3d.getZ
    final PIDController PIDLateral = new PIDController(0, 0, 0); // Pose3d.getX
    final PIDController PIDAngle = new PIDController(0, 0, 0); // Pose3d.getRotations.getY

    
    Pose3d DesiredPose;
    LimelightHelpers.LimelightTarget_Fiducial CurrentTarget;
    Pose3d CurrentPose;
    Vision.ETagType CurrentTagType;
    
    
    public AlignToTagCommand(boolean finishWhenClose, SwerveDrivetrain drivetrain)
    {
        FinishWhenClose = finishWhenClose;
        Drivetrain = drivetrain;
    }
    
    boolean UpdateCurrent()
    {
        CurrentTarget = Vision.GetBestTarget();
        if (CurrentTarget == null) 
        {
            CurrentPose = null;
            CurrentTagType = null;
            return false;
        }
        
        CurrentPose = CurrentTarget.getRobotPose_TargetSpace();
        CurrentTagType = Vision.GetTagTypeForTarget(CurrentTarget);

        // -- We lock in our desired pose the first time we get a target, then drive towards it.
        //    There's no need to re-calculate the desired pose each time
        if (DesiredPose == null)
        {
            // TODO: Handle being offset from the speaker and making a different pose to come in from the appropriate side
            DesiredPose = new Pose3d(0, 0, CurrentTagType.DistanceOffset, new Rotation3d(0, 0, 0));
        }
        
        return true;
    }
    
    void CalculateDesiredPose()
    {
        
    }
    
    @Override
    public void initialize() { }

    @Override
    public void execute()
    {
        if (!UpdateCurrent())
        {
            Drivetrain.setControl(RobotContainer.Get().GetDefaultDriveRequest());
            return;
        }
        
        double distance = CurrentPose.getTranslation().getZ();
        double lateral = CurrentPose.getTranslation().getX();
        double angle = CurrentPose.getRotation().getY();
        
        double outDistance = PIDDistance.calculate(distance, DesiredPose.getZ());
        double outLateral = PIDLateral.calculate(lateral, DesiredPose.getX());
        double outAngle = PIDAngle.calculate(angle, DesiredPose.getRotation().getY());

        SmartDashboard.putNumber("Target.InLateral", lateral);
        SmartDashboard.putNumber("Target.InDistance", distance);
        SmartDashboard.putNumber("Target.InAngle", angle);
        
        SmartDashboard.putNumber("Target.outLateral", outDistance);
        SmartDashboard.putNumber("Target.outDistance", outLateral);
        SmartDashboard.putNumber("Target.outAngle", outAngle);
        
        double rotationalOffset = 0.05;

        Drivetrain.setControl(
            driveRobotCentric
                .withVelocityX(-outDistance)
                .withVelocityY(outLateral)
                .withRotationalRate(outAngle)
        );
    }

    @Override
    public boolean isFinished()
    {
        if (!FinishWhenClose) { return false; }
        if (CurrentTarget == null) { return false; }
        
        return PIDDistance.atSetpoint()
            && PIDLateral.atSetpoint()
            && PIDAngle.atSetpoint();
    }
}
