package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
/**
 * AutoTest Class
 */
public class AutoLeftMulti extends OpMode {

    Thunderbot_2022 robot = new Thunderbot_2022();
    boolean done = false;
    int state = 0;

    // All of these values (A-F) work on the strafing to the right
    // stepA is a drive
    double stepA = 147.5; // was 120
    // stepB is a turn
    double stepB = 90;
    // stepC is a drive
    double stepC = 0.5;
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
            stepF = 60;
            stepFPower = -0.5;
        }
        else if (theZone == 2)
        { // set the number to the value of half black
            stepF = 0;
            stepFPower = 0;
        }
        else
        { // set the number to the value of white
            stepF = 60;
            stepFPower = 0.5;
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
                    done = robot.gyroDrive(0, stepA, 0.5);
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
                    done = robot.drive(179, 30.5, 0.2);
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
                    done = robot.turnTo(stepB, 0.5);
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
                    done = robot.turnTo(92.5, 0.5);
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
                    done = true;
                    done = robot.gyroDrive(92.5, stepC, -0.3); // was 43.5
                    telemetry.addData("case 2", "is started");
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            /////////////
            // Linear Slide Movement
            /////////////
                /*
            case 6:
                if (!done) {
                    done = robot.armstrong.liftUpDistance(25, 0.4);
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
                    done = robot.armstrong.liftDownDistance(28, 0.5) || (getRuntime() > 2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;*/

            case 6:
                if (!done) {
                    done = robot.turnTo( 0, 0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 7:
                if (!done) {
                    done = robot.gyroDrive(0, 27.5, 0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 8:
                if (!done) {
                    done = robot.turnTo( 100, 0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 9:
                if (!done) {
                    done = robot.gyroDrive( 90,35, 0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 10:
                if (!done) {
                    done = robot.armstrong.elbowMove(0.535);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 11:
                if (!done) {
                    done = robot.gyroDrive( 90, 65,-0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;

            case 12:
                if (!done) {
                    done = robot.turnTo( -90, 0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 13:
                if (!done) {
                    done = robot.gyroDrive(-90, 15, 0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
                
            /////////////
            // add in the picking up of the cone
            ////////////



//            case 16:
//                if (!done) {
//                    done = robot.gyroDrive(-135, 22.5, -0.5);
//                } else {
//                    robot.stop();
//                    done = false;
//                    state++;
//                }
            ////////////
            // Parking
            ////////////
                /*            case 13:
                if (!done) {
                    done = robot.gyroDrive(95, 1, 0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 14:
                if (!done) {
                    done = robot.drive(90, 28, 0.5);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 15:
                if (!done) {
                    done = robot.gyroDrive(90, stepF, stepFPower);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;*/

            default:
                break;
        }
        telemetry.addData("State", state);
    }
}
