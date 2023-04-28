package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
/**
 * AutoTest Class
 */
public class AutoRightHigh extends OpMode {

    Thunderbot_2022 robot = new Thunderbot_2022();
    boolean done = false;
    int state = 0;
    // Defining the Variables that will be used for the steps in the autonomous

    // All of these values (A-F) work on the strafing to the right
    // stepA is a drive
    double stepA = 152; // was 120
    // stepB is a turn
    double stepB = -92.5;
    // stepC is a drive
    double stepC = 5.5;
    // stepD is a drive
    double stepD = 5;
    // stepE is a turn
    double stepE = 0;
    // stepF is a strafe
    double stepF = 42.5;
    double stepFPower = 0;
    int theZone = 2;

    @Override
    public void init()
    {
        robot.init(hardwareMap, telemetry, true);
    }

    @Override
    public void init_loop()
    {
        theZone = robot.vision.getSignalZone();
        telemetry.addData("Zone", theZone);
    }

    @Override
    public void start()
    {

        if (theZone == 1)
        { // set the number to the value of black
            stepF = 42.5;
            stepFPower = 0.3;
        }
        else if (theZone == 2)
        { // set the number to the value of half black
            stepF = 0;
            stepFPower = 0;
        }
        else if (theZone == 3)
        { // set the number to the value of white
            stepF = 42.5;
            stepFPower = -0.3;
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
        robot.update();
        telemetry.addData("Zone", theZone);

        switch (state)
        {
            case 0:
                if (!done) {
                    done = robot.armstrong.elbowMove(0.375);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 1:
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
            case 2:
                if (!done) {
                    done = robot.drive(179, 25.5, 0.2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 3:
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
            case 4:
                if (!done)
                {
                    // stepB is 45
                    done = robot.turnTo(-92.5, 0.2);
                    telemetry.addData("case 1", "is started");
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
                    // stepC is 10y
                    done = robot.gyroDrive(-90, stepC, -0.2); // was 43.5
                    telemetry.addData("case 2", "is started");
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 6:
                if (!done) {
                    done = robot.armstrong.liftUpDistance(42, 0.4);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 7:
                if (!done) {
                    done = robot.armstrong.wristMove(0.135);
                    done = done && (getRuntime() > 2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 8:
                if (!done) {
                    robot.armstrong.liftDownDistance(3, 0.4);
                    done = robot.armstrong.wristMove(0);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 9:
                if (!done) {
                    done = robot.armstrong.clawMove(0.3);
                    done = done && (getRuntime() > 1);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 10:
                if (!done) {
                    done = robot.armstrong.wristMove(0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 11:
                if (!done) {
                    done = robot.armstrong.elbowMove(0.535) || (getRuntime() > 1);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 12:
                if (!done) {
                    done = robot.armstrong.liftDownDistance(43, 0.5) || (getRuntime() > 2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 13:
                if (!done) {
                    done = robot.gyroDrive(-90, 3, 0.3);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 14:
                if (!done) {
                    done = robot.drive(-90, 75, 0.3);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 15:
                if (!done) {
                    done = robot.gyroDrive(-90, stepF, stepFPower);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            default:
                break;

        }
        telemetry.addData("State", state);
    }
}
