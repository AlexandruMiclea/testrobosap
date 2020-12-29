package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.localizer.vision.TensorFlowThread;
import org.firstinspires.ftc.teamcode.drive.tank.Robot;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@TeleOp(name = "Concept: TensorFlowThread Test", group = "Concept")
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
