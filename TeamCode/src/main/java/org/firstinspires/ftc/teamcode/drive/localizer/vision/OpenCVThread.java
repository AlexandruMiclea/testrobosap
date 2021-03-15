package org.firstinspires.ftc.teamcode.drive.localizer.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVThread extends Thread{

    OpenCvInternalCamera phoneCam;
    RingStackDeterminationPipeline pipeline;

    //Constructor
    public OpenCVThread(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingStackDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
    }

    //Destructor
    @Override
    public void finalize() throws Throwable {
        super.finalize();
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();
    }

    public int analysis;
    public RingStackDeterminationPipeline.RingPosition numberOfRing;

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

        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();
    }

}

