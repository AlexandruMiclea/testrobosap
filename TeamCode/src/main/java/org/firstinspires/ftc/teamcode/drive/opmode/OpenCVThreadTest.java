package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.localizer.vision.OpenCVThread;
import org.firstinspires.ftc.teamcode.drive.localizer.vision.RingStackDeterminationPipeline;
import org.firstinspires.ftc.teamcode.drive.localizer.vision.TensorFlowThread;

import java.util.List;

@TeleOp(name = "OpenCV Thread Test", group = "Concept")
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
        timer.reset();
        timer.startTime();

        while (timer.milliseconds() < MAX_MILISECONDS){
            ring = openCV.getRingPosition();
        }

        if(ring == RingStackDeterminationPipeline.RingPosition.FOUR){
            test = "ai cam multe tovarase";
        } else if(ring == RingStackDeterminationPipeline.RingPosition.ONE){
            test = "numai unu coane";
        } else {
            test = "Nema fra";
        }

        while(opModeIsActive()){
            telemetry.addData("timer", timer.milliseconds());
            telemetry.addData("number of ring: ", openCV.getRingPosition());
            telemetry.addData("analysis: ", openCV.getAnalysis());
            telemetry.addData("ce am eu:", test);
            telemetry.update();
        }

        try {
            openCV.finalize();
        } catch (Throwable throwable) {
            throwable.printStackTrace();
        }

    }
}