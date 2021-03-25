package org.firstinspires.ftc.teamcode.drive.localization.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCVThread extends Thread{

    OpenCvWebcam Webcam;
    RingStackDeterminationPipeline pipeline;

    private int analysis;
    private RingStackDeterminationPipeline.RingPosition numberOfRing;


    //Constructor
    public OpenCVThread(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "CAMERANAME"); //TODO get camera name
        Webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new RingStackDeterminationPipeline();
        Webcam.setPipeline(pipeline);

        Webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    //Destructor
    @Override
    public void finalize() throws Throwable {
        super.finalize();
        Webcam.stopStreaming();
        Webcam.closeCameraDevice();
    }

    public int getAnalysis()
    {
        return analysis;
    }

    public RingStackDeterminationPipeline.RingPosition getRingPosition(){
        return numberOfRing;
    }


    //Dis the RUN method
    @Override
    public void run() {
        while(this.isAlive()){
            this.analysis = pipeline.getAnalysis();
            this.numberOfRing = pipeline.getPosition();
        }

        Webcam.stopStreaming();
        Webcam.closeCameraDevice();
    }

}

