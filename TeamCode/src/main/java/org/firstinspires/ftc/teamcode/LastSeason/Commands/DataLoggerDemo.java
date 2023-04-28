package org.firstinspires.ftc.teamcode.LastSeason.Commands;

/**
 * Created by Olavi Kamppari on 11/3/2015.
 */

/**
 * Demonstrate the simultaneous usage data loggers.
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class DataLoggerDemo extends OpMode {

    private DataLogger dataLogger;

    public void init() {

        dataLogger = new DataLogger("DlDemo_lt");
        dataLogger.addField("Time");
        dataLogger.addField("Left Encoder Distance");
        dataLogger.addField("Right Encoder Distance");
        dataLogger.addField("Heading Angle");
        dataLogger.newLine();
    }

    public  void init_loop() {
    }


    public void start() {
        resetRuntime();
    }


    @Override
    public void loop() {
        dataLogger.addField(getRuntime());
        dataLogger.addField(Math.random());
        dataLogger.addField(10 + Math.random());
        dataLogger.addField(20 + Math.random());
        dataLogger.newLine();
    }

    public void stop() {
        dataLogger.closeDataLogger();
    }

}
