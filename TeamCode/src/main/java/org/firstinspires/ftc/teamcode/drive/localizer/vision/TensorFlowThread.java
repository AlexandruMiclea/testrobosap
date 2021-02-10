package org.firstinspires.ftc.teamcode.drive.localizer.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class TensorFlowThread extends Thread {

    public static final String TAG = "TensorThread";

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AYlEu/7/////AAABmXB1kirNm0vlrZa4DCCmkis6ZNJkEkHGNYjIfoKWcK+yxnJOhuC4Lw3B63L+Y5vrSoTsr1mEe6bvGcMR8Hg+v1Z1Cih0IrBRHdIfrrg6lfa723ft/unZOKgck3ftCj8gWuiM89d+A4smkenUI5P/HXMKMGKCk4xxv5of9YNSX8r4KFO8lD+bqYgnP+GVXzD/TwQo7Dqer3bf0HVbOqP6j6HREHAZdP6Idg/JwyRG8LSdC6ekTwogxCWsuWiaUhuC8uAQ4r/ZfJykZpXYCxhdcLwMM4OaUXkUAPuUenzxlL8MXkwOhsDfqiQNEfSB00BodWKq28EC6cc+Vsko8r9PreeU6jCYR4d84VK8uBFLGaJx";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //Constructor
    public TensorFlowThread(HardwareMap hardwareMap) {
        activateTfod(hardwareMap);
    }

    //Destructor
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        closeTfod();
    }

    public int recognitionsSize;
    public String label;
    public List<Recognition> recognitions = new ArrayList<>();

    public int getRecognitionsSize() {
        return recognitionsSize;
    }

    public String getLabel(){
        return label;
    }

    public List<Recognition> getRecognitions(){
        return recognitions;
    }

    //Dis the RUN method
    @Override
    public void run() {
        while(this.isAlive()){
            if (tfod != null) {
                this.recognitions = tfod.getUpdatedRecognitions();
                if (recognitions != null) {
                    this.recognitionsSize = recognitions.size();
                    int i = 0;
                    for (Recognition recognition : recognitions) {

                        this.label = recognition.getLabel();
                    }
                }
            }

        }

        closeTfod();
    }


    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f; //TODO modify
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void activateTfod(HardwareMap hardwareMap){
        initVuforia();

        initTfod(hardwareMap);

        if (tfod != null) {
            tfod.activate();
        }
    }

    private void closeTfod(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}


