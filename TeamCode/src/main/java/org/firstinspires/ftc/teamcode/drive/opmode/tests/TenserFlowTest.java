package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.localization.vision.TensorFlowThread;

import java.util.List;

@TeleOp(name = "Concept: TensorFlowThread Test", group = "test")
@Disabled
public class TenserFlowTest extends LinearOpMode {

    public TensorFlowThread tensorFlow ;

    @Override
    public void runOpMode() {
        tensorFlow = new TensorFlowThread(hardwareMap);

        waitForStart();

        tensorFlow.start();

        while(opModeIsActive()){
            List<Recognition> recognitions = tensorFlow.getRecognitions();
            if (recognitions != null) {
                telemetry.addData("# Object Detected", tensorFlow.getRecognitionsSize());
                int i = 0;
                for (Recognition recognition : recognitions) {
                    telemetry.addData("label", tensorFlow.getLabel());
                }
            }
            telemetry.update();
        }
    }
}
