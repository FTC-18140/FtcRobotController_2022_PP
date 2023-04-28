package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class AprilEyes {

Telemetry telemetry;
    OpenCvCamera camera;
    AprilEyesDetectionPipeline aprilEyesDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


    public void init(HardwareMap hardwareMap, Telemetry telem){
        telemetry =  telem;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilEyesDetectionPipeline = new AprilEyesDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilEyesDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode)

            {

            }
        });
    }

    public int getSignalZone() {
        if ( aprilEyesDetectionPipeline != null) {
            return aprilEyesDetectionPipeline.getSignalZone();
        } else {
            return 0;
        }
    }

    public void stopCamera()
    {
        try
        {
            camera.stopStreaming();
        }
        catch (Exception e)
        {
            telemetry.addData("Camera is not streaming.", 0);
        }
    }
}
