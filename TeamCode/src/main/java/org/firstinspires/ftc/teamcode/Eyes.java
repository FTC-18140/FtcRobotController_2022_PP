package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;

public class Eyes
{

    OpenCvCamera phoneCam;
    StageSwitchingPipeline stageSwitchingPipeline;
    Telemetry telemetry;
    public void init(HardwareMap hardwareMap,Telemetry telem)
    {
        try {
            telemetry = telem;
            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

            stageSwitchingPipeline = new StageSwitchingPipeline();
            stageSwitchingPipeline.telem = telemetry;
            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    phoneCam.setPipeline(stageSwitchingPipeline);
                    phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); //width was on 640
                }

                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        } catch (Exception e) {
            telemetry.addData("camera", "not found");
        }

    }
    /*
     * With this pipeline, we demonstrate how to change which stage of
     * is rendered to the viewport when the viewport is tapped. This is
     * particularly useful during pipeline development. We also show how
     * to get data from the pipeline to your OpMode.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat smallerMatrix = new Mat();
        String answer = new String();
        int signalZone;
        Scalar green = new Scalar(0,250,0);
        Scalar blue = new Scalar( 0, 0, 255);
        Scalar yellow = new Scalar(255, 255, 0);
        Telemetry telem;
        int rowStart = 150;
        int rowEnd = 250;
        int colStart = 280;
        int colEnd = 350;
        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * This pipeline finds the signal and identifies what zone it indicates.
             */

            // Step 1 - identify the rectangle in the image that we want to study
            Imgproc.rectangle( input, new Point( colStart,rowStart), new Point( colEnd, rowEnd), green, 4); // x = 100 x = 300

            // Step 2 - create a smaller, submatrix that only contains the pixels in the rectangle we are studying
            smallerMatrix = input.submat(rowStart,rowEnd, colStart, colEnd);
//            you can use the submat( .. ) method such as
//            smallerMatrix = input.submat( ...... );


            // Step 3 - apply the OpenCV conversions, filters, thresholds, etc. to the submatrix

//            Examples -- these will not work for our season -- but you can see the the kinds of things we might do
              Mat grayMatrix = new Mat();
              Mat thresholdMat =new Mat();
             Imgproc.cvtColor(smallerMatrix, grayMatrix, Imgproc.COLOR_RGB2GRAY);
//            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(smallerMatrix, thresholdMat, 150, 255, Imgproc.THRESH_BINARY);


            // Step 4 - add the values of the pixels in our submatrix
           double sumColors = Core.sumElems(thresholdMat).val[0];
         //  telem.addData("sum: ",sumColors);
            // Step 5 - store the answer in the signalZone variable.  Also can write it on the screen.

//            signalZone = <our answer goes here>;
            if (sumColors > 1200000) {
                signalZone = 2;
            }
            else if (sumColors < 500000) {
                signalZone = 1;
            }
            else {
                signalZone = 3;
            }

            if (signalZone == 1)
            {
                answer = "ONE";
            }
            else if ( signalZone == 2)
            {
                answer = "TWO";
            }
            else
            {
                answer = "THREE";
            }
            Imgproc.putText( input, String.valueOf(sumColors), new Point( 10, 400), FONT_HERSHEY_SIMPLEX, 2, yellow, 3);
            Imgproc.putText( input, answer, new Point( 10, 450), FONT_HERSHEY_SIMPLEX, 2, yellow, 3);

           // telem.addData("answer", answer);
            return input;
        }

    }

    /**
     * Returns the zone number for the zone identified by the signal in the OpenCV code
     * @return int zone number
     */
    int getSignalZone()
    {
        return stageSwitchingPipeline.signalZone;
    }

}
