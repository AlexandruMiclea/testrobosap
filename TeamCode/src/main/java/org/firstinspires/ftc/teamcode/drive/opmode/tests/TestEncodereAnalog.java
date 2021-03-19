package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.localization.localizers.AnalogEncoderLocalizer;

@TeleOp(name = "Test Encodere Analog", group = "test")
public class TestEncodereAnalog extends LinearOpMode {
    public AnalogEncoderLocalizer encodere;

    @Override
    public void runOpMode() {
        encodere = new AnalogEncoderLocalizer(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Index: ", encodere.getIndex());
            telemetry.addData("Voltaje totale cu index: ", encodere.getTotalVoltagesWithIndex());
            telemetry.addData("Voltaje totale in inch: ", encodere.getWheelPositions());
            telemetry.update();
        };
    }
}
