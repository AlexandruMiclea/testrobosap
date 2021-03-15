package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.localizer.AnalogEncoderLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.AnalogEncoderLocalizerMR;

@TeleOp(name = "Test Encodere Analog", group = "Concept")
public class TestEncodereAnalog extends LinearOpMode {
    public AnalogEncoderLocalizerMR encodere;

    @Override
    public void runOpMode() {
        encodere = new AnalogEncoderLocalizerMR(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Voltaje: ", encodere.getVoltages());
            telemetry.addData("Derivate: ", encodere.getDerivatives());
            telemetry.addData("Voltaje totale: ", encodere.getTotalVoltages());
            telemetry.addData("Index: ", encodere.getIndex());
            telemetry.addData("Voltaje totale cu index: ", encodere.getTotalVoltagesWithIndex());
            telemetry.addData("Voltaje totale in inch: ", encodere.getWheelPositions());
//            telemetry.addData("val random decimal test: ", encodere.toThreeDec(1.0240000000000012));
            telemetry.update();
        };
    }
}
