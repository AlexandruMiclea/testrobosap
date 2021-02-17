package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.localizer.AnalogEncoderLocalizer;

@TeleOp(name = "Test Encodere Analog", group = "Concept")
public class TestEncodereAnalog extends LinearOpMode {
    public AnalogEncoderLocalizer encodere;

    @Override
    public void runOpMode() {
        encodere = new AnalogEncoderLocalizer(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Voltaje: ", encodere.getVoltages());
            telemetry.addData("Derivate: ", encodere.getDerivatives());
            telemetry.addData("Voltaje totale: ", encodere.getWheelPositions());
            telemetry.update();
        };
    }
}
