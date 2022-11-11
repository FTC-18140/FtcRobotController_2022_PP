package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
/**
 * AutoTest Class
 */
public class AutoRightSide extends OpMode
{
    Thunderbot_2022 robot = new Thunderbot_2022();
    boolean done = false;
    int state = 0;

    double stepA = 45; // 45 What?
    double stepB = -45; // 45 What?
    double stepC = 0;
    double stepD = 8;
    double stepE = 45;
    double stepF = 41;
    int theZone = 2;

    @Override
    public void init()
    {
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void init_loop()
    {
        theZone = robot.vision.getSignalZone();
    }

    @Override
    public void start()
    {

        if (theZone == 1)
        { // set the number to the value of black
            stepF = -41;
        }
        else if (theZone == 2)
        { // set the number to the value of half black
            stepF = 0;
        }
        else if (theZone == 3)
        { // set the number to the value of white
            stepF = 41;
        }
        else
        {
            telemetry.addData("Unknown Case", theZone);
        }

        telemetry.addData("Robot Location: ", robot.updateHeading());
    }

    @Override
    public void loop()
    {
        switch (state)
        {
            case 0:
                if (!done)
                {
                    done = robot.gyroDrive(0, stepA, 0.2);
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 5:
                if (!done)
                {
                    done = robot.drive(90, stepF, 0.2);
                }
                else
                {
                    robot.stop();
                    done = false;
                    telemetry.addData("program", "ended");
                    state++;
                }
                break;
            default:
                break;
        }
    }
}