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

    // stepA is a drive
    double stepA = 45;
    // stepB is a turn
    double stepB = 45;
    // stepC is a drive
    double stepC = 5;
    // stepD is a drive
    double stepD = 5;
    // stepE is a turn
    double stepE = 0;
    // stepF is a strafe
    double stepF = 65;
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
                    // stepA is 45
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
                    // stepB is 45
                    done = robot.turnTo(stepB, 0.2);
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
                    // stepC is 8
                    done = robot.gyroDrive(45, stepC, 0.2);
                    telemetry.addData("case 2", "is started");
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
                    // stepD is 8
                    done = robot.drive(180, stepD, 0.2);
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
                    // stepE is 0
                    done = robot.turnTo(stepE, 0.2);
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
                    // stepF is 65
                    done = robot.drive(90, stepF, 0.4);
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
        telemetry.addData("State", state);
    }
}
