package friarLib2.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class RumbleCommand extends WaitCommand
{
    public enum ERumbleTarget
    {
        Driver,
        Operator,
        Both;
    }

    private ERumbleTarget Target = ERumbleTarget.Both;
    private GenericHID.RumbleType Type = GenericHID.RumbleType.kBothRumble;
    private double Strength = 1;

    private RumbleCommand(ERumbleTarget target, double duration)
    {
        super(duration);
        Target = target;
    }

    public RumbleCommand WithType(GenericHID.RumbleType type)
    {
        Type = type;
        return this;
    }

    public RumbleCommand WithStrength(double strength)
    {
        Strength = strength;
        return this;
    }

    @Override
    public void initialize()
    {
        if (Target == ERumbleTarget.Driver || Target == ERumbleTarget.Both)
        {
            RobotContainer.Driver.getHID().setRumble(Type, Strength);
        }

        if (Target == ERumbleTarget.Operator || Target == ERumbleTarget.Both)
        {
            RobotContainer.Operator.getHID().setRumble(Type, Strength);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        super.end(interrupted);

        if (Target == ERumbleTarget.Driver || Target == ERumbleTarget.Both)
        {
            RobotContainer.Driver.getHID().setRumble(Type, 0);
        }

        if (Target == ERumbleTarget.Operator || Target == ERumbleTarget.Both)
        {
            RobotContainer.Operator.getHID().setRumble(Type, 0);
        }
    }


    public static RumbleCommand Once(ERumbleTarget target, double duration)
    {
        return new RumbleCommand(target, duration);
    }

    public static RumbleCommand Once(ERumbleTarget target, double duration, GenericHID.RumbleType type, double strength)
    {
        return new RumbleCommand(target, duration)
            .WithType(type)
            .WithStrength(strength);
    }

    private static Command MakePulseSequence(RumbleCommand cmd, double pauseDuration, int pulseCount)
    {
        var state = new Object()
        {
            Integer count = 0;
        };

        return
            Commands.repeatingSequence(
                cmd,
                Commands.waitSeconds(pauseDuration),
                Commands.runOnce(() -> state.count++)
            )
            .until(() -> state.count >= pulseCount);
    }

    public static Command Pulse(ERumbleTarget target, double rumbleDuration, double pauseDuration, int pulseCount)
    {
        return MakePulseSequence(Once(target, rumbleDuration), pauseDuration, pulseCount);
    }
    public static Command Pulse(ERumbleTarget target, double rumbleDuration, double pauseDuration, int pulseCount, GenericHID.RumbleType type, double strength)
    {
        return MakePulseSequence(Once(target, rumbleDuration, type, strength), pauseDuration, pulseCount);
    }
}
