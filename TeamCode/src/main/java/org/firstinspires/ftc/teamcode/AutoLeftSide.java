package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
/**
 * AutoTest Class
 */
public class AutoLeftSide extends OpMode
{
    Thunderbot_2022 robot = new Thunderbot_2022();
    boolean done = false;
    int state = 0;
    // Defining the Variables that will be used for the steps in the autonomous
    double stepA = 45;
    double stepB = -45;
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
            case 1:
                if (!done)
                {
                    done = robot.turn(stepB, -0.2);
                    telemetry.addData("case 1", "is started");
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 2:
                if (!done)
                {
                    done = robot.gyroDrive(90, stepC, 0.2);
                    telemetry.addData("case 2", "is started");
                    done = true;
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 3:
                if (!done)
                {
                    done = robot.gyroDrive(-45, stepD, -0.2);
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 4:
                if (!done)
                {
                    done = robot.turn(stepE, 0.2);
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
