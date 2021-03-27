package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@TeleOp(group = "driver", name = "Test Tickuri Middle Encoder")
public class TestDcMotorEncoder extends LinearOpMode {

    public Encoder middleEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        middleEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "middleEncoder"));
        middleEncoder.setDirection(Encoder.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("encoder ticks:", middleEncoder.getCurrentPosition());
            telemetry.addData("supposed ticks per rev: ", middleEncoder.getTicksPerRev());
            telemetry.update();
        }

    }
}
