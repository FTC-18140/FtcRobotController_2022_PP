package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.LastSeason.Commands.CommandOpModes.AutoLegendary;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.Subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase {
    private final double myAngle;
    private final ClawSubsystem myClaw;
    String name = "default";


    public ClawCommand(double angle, ClawSubsystem claw, String theName) {
        myAngle = angle;
        myClaw = claw;
        addRequirements(myClaw);
        name = theName;
    }

    @Override
    public void initialize() {
        if (name == "coneOne") {
            AutoLegendary.coneOneDropped = true;
        } else if (name ==  "coneTwo") {
            AutoLegendary.coneTwoDropped = true;
        } else if (name == "coneThree") {
            AutoLegendary.coneThreeDropped = true;
        }
    }


    @Override
    public void execute() {
        myClaw.clawMove(myAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}