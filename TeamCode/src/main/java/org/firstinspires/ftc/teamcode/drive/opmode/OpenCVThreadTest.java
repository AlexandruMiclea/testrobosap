package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.localizer.vision.OpenCVThread;
import org.firstinspires.ftc.teamcode.drive.localizer.vision.TensorFlowThread;

import java.util.List;

@TeleOp(name = "OpenCV Thread Test", group = "Concept")
public class OpenCVThreadTest extends LinearOpMode{
    public OpenCVThread openCV;

    @Override
    public void runOpMode() {
        openCV = new OpenCVThread(hardwareMap);

        waitForStart();

        openCV.start();

        while(opModeIsActive()){
            telemetry.addData("# Object Detected", openCV.getAnalysis());
            telemetry.addData("label", openCV.getRingPosition());

            telemetry.update();
        }

        try {
            openCV.finalize();
        } catch (Throwable throwable) {
            throwable.printStackTrace();
        }
    }
}