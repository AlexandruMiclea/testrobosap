package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.localization.vision.OpenCVThread;
import org.firstinspires.ftc.teamcode.drive.localization.vision.RingStackDeterminationPipeline;

@TeleOp(name = "OpenCV Thread Test", group = "test")
public class OpenCVThreadTest extends LinearOpMode{
    public OpenCVThread openCV;
    public RingStackDeterminationPipeline.RingPosition ring;
    public ElapsedTime timer;
    public static int MAX_MILISECONDS = 4000;
    public String test = "valoare initiala";

    @Override
    public void runOpMode() {
        openCV = new OpenCVThread(hardwareMap);
        ring = RingStackDeterminationPipeline.RingPosition.NONE;

        openCV.start();

        timer = new ElapsedTime();
        telemetry.addData("has initialised", "yes");
        telemetry.update();

        waitForStart();

        timer.startTime();

        while (openCV.getAnalysis() == 0 && timer.milliseconds() < MAX_MILISECONDS){
            ring = openCV.getRingPosition();
            telemetry.addData("ho bos", "");
            telemetry.update();

        }

        while(opModeIsActive()){
            //telemetry.addData("timer", timer.milliseconds());
            telemetry.addData("number of ring: ", openCV.getRingPosition());
            telemetry.addData("analysis: ", openCV.getAnalysis());
            telemetry.update();
        }

        try {
            openCV.finalize();
        } catch (Throwable throwable) {
            throwable.printStackTrace();
        }

    }
}