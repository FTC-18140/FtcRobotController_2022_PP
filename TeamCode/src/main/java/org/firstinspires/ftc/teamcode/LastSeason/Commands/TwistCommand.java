package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.LastSeason.Commands.Subsystems.ArmSubsystem;

public class TwistCommand extends CommandBase
{
    private final double myAngle;
    private final ArmSubsystem myArm;


    public TwistCommand(double angle, ArmSubsystem arm )
    {
        myAngle = angle;
        myArm = arm;
        addRequirements(myArm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        myArm.armTwist(myAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
