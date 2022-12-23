package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
/**
 * AutoTest Class
 */
public class AutoLeftSide extends OpMode {

    Thunderbot_2022 robot = new Thunderbot_2022();
    boolean done = false;
    int state = 0;
    // Defining the Variables that will be used for the steps in the autonomous

//    ** All of these values (A-F) work on the strafing to the left **
//    double leftStepA = 45;
//    double leftStepB = 45;
//    double leftStepC = 5;
//    double leftStepD = 5;
//    double leftStepE = 0;
//    double leftStepF = 50;
//
//   ** All of these values (A-F) work on staying still **
//    double centerStepA = 45;
//    double centerStepB = 45;
//    double centerStepC = 5;
//    double centerStepD = 5;
//    double centerStepE = 0;
//    double centerStepF = 0;

    // All of these values (A-F) work on the strafing to the right
    // stepA is a drive
    double stepA = 72;
    // stepB is a turn
    double stepB = 45;
    // stepC is a drive
    double stepC = 13;
    // stepD is a drive
    double stepD = 5;
    // stepE is a turn
    double stepE = 0;
    // stepF is a strafe
    double stepF = 50;
    double stepFPower = 0.6;
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
            stepF = 50;
            stepFPower = -0.4;
        }
        else if (theZone == 2)
        { // set the number to the value of half black
            stepF = 0;
        }
        else if (theZone == 3)
        { // set the number to the value of white
            stepF = 50;
            stepFPower = 0.4;
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

        switch (state)
        {
            case 0:
                if (!done)
                {
                    // stepA is 45
                    done = robot.gyroDrive(0, stepA, 0.3);
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 1:
                if (!done) {
                    done = robot.drive(179, 15, 0.2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 2:
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
            case 3:
                if (!done)
                {
                    // stepC is 10y
                    done = robot.gyroDrive(43.5, stepC, 0.2);
                    telemetry.addData("case 2", "is started");
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 4:
                if (!done) {
                    done = robot.armstrong.liftUpDistance(20, 0.4);

                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 5:
                if (!done) {
                    done = robot.armstrong.elbowLowerDistance(2, 0.4);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 6:
                if (!done) {
                    done = robot.armstrong.wristMove(0);
                    done = done && (getRuntime() > 2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 7:
                if (!done) {
                    done = robot.armstrong.liftDownDistance(3, 0.4);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 8:
                if (!done) {
                    done = robot.armstrong.clawMove(0.5);
                    done = done && (getRuntime() > 1);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 9:
                if (!done) {
                    done = robot.armstrong.wristMove(0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 10:
                if (!done) {
                    done = robot.armstrong.elbowRaiseDistance(35, 0.7) || (getRuntime() > 1);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                    resetRuntime();
                }
                break;
            case 11:
                if (!done) {
                    done = robot.armstrong.liftDownDistance(17, 0.5) || (getRuntime() > 2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 12:
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
            case 13:
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
            case 14:
                if (!done)
                {
                    // stepF is 65
                    done = robot.drive(90, stepF, stepFPower); // the power changes to a negative when it is to the left
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
