package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase
{
    private static final double ArmTolerance = 15.0 / 360.0;
    private static final double LowerLimit = -0.12;
    private static final double UpperLimit = 0.28;

    private double ManualArmControlTarget = 0;

    public enum EArmPosition {
        Stowed(LowerLimit),
        Shoot_speaker(-0.058), //TODO: Og .078
        Shoot_podium(LowerLimit), //TODO: tune when added
        Shoot_wing(LowerLimit), //TODO: tune when added
        Climb_FirstPos(UpperLimit),
        Amp(0.169),
        Trap(LowerLimit),
        Source(-0.05);

        public final double Rotations;

        EArmPosition(double rotations) {
            Rotations = rotations;
        }
    }

    // -- Motors
    private TalonFX LeftMotor;
    private TalonFX RightMotor;
    private final MotionMagicExpoTorqueCurrentFOC PoseRequest =
        new MotionMagicExpoTorqueCurrentFOC(LowerLimit)
            .withSlot(0);
    private final PositionTorqueCurrentFOC ClimbRequest =
        new PositionTorqueCurrentFOC(LowerLimit)
            .withSlot(1);


    public ArmSubsystem()
    {
        LeftMotor = CreateMotor(Constants.CanivoreBusIDs.ArmLeft.GetID());

        RightMotor = CreateMotor(Constants.CanivoreBusIDs.ArmRight.GetID());
        RightMotor.setControl(new Follower(Constants.CanivoreBusIDs.ArmLeft.GetID(), true));

        LeftMotor.setControl(PoseRequest.withPosition(EArmPosition.Stowed.Rotations));
    }



    public double GetArmPosition() { return LeftMotor.getPosition().getValue(); }



    private TalonFX CreateMotor(int deviceID)
    {
        var motor = new TalonFX(deviceID, Constants.CanivoreBusIDs.BusName);

        var configs = new TalonFXConfiguration();

        configs.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(300)
                .withKI(0)
                .withKD(65)
                .withKS(0)
                .withKA(0)
                .withKV(0)
                .withKG(8));

        configs.withSlot1(
            new Slot1Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(2000)
                .withKI(0) //TODO: add some KI to make sure we fully climb (probably about 150ish)
                .withKD(65)
                .withKS(8)
                .withKA(0)
                .withKV(0)
                .withKG(27));

        configs.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(0)
                .withMotionMagicCruiseVelocity(5)
                .withMotionMagicExpo_kA(3)
                .withMotionMagicExpo_kV(5)
                .withMotionMagicJerk(1000));

        configs.withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(117.6));

        configs.withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

        configs.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(UpperLimit)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(LowerLimit));

        motor.getConfigurator().apply(configs);

        motor.setPosition(LowerLimit);
        return motor;
    }

    public Command Command_SetPosition(EArmPosition position) {
        return Command_GoToPosition(position.Rotations);
    }

    public Command Command_Climb()
    {
        return runOnce(() -> LeftMotor.setControl(ClimbRequest.withPosition(LowerLimit)));
    }

    public Command Command_ZeroArmEncoder()
    {
       return runOnce(() -> LeftMotor.setPosition(LowerLimit)).ignoringDisable(true);
    }

    public Command Command_ManualArmControl()
    {
        return runOnce(() -> ManualArmControlTarget = LeftMotor.getPosition().getValue())
            .andThen(run(() ->
            {
                double y = RobotContainer.Operator.getLeftY() * 0.001;
                if (Math.abs(y) < 0.001) { return; }

                ManualArmControlTarget = MathUtil.clamp(ManualArmControlTarget + y, LowerLimit, UpperLimit);
                //LeftMotor.setControl(PoseRequest.withPosition(ManualArmControlTarget));
                LeftMotor.setControl(ClimbRequest.withPosition(ManualArmControlTarget));
            }));
    }

    public Command Command_SetNeutralMode(NeutralModeValue mode)
    {
        return runOnce(() -> {
            LeftMotor.setNeutralMode(mode);
            RightMotor.setNeutralMode(mode);
        }).ignoringDisable(true);
    }

    public Command Command_GoToPosition(double armPosition)
    {
        var pos = MathUtil.clamp(armPosition, LowerLimit, UpperLimit);
        return
            run(() ->
                {
                    SmartDashboard.putNumber("Arm.Target", pos);
                    LeftMotor.setControl(PoseRequest.withPosition(pos));
                })
                .until(() ->
                       {
                           double actualRotation = LeftMotor.getPosition().getValue();
                           SmartDashboard.putNumber("Arm.Error", pos - actualRotation);
                           return MathUtil.isNear(pos, actualRotation, ArmTolerance);
                       });
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Arm.PositionL", LeftMotor.getPosition().getValue());
        SmartDashboard.putNumber("Arm.PositionR", RightMotor.getPosition().getValue());
    }

}