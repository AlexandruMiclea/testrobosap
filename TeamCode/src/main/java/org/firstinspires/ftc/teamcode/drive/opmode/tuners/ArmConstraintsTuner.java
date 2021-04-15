package org.firstinspires.ftc.teamcode.drive.opmode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.subsystems.WobbleArm;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

/*
* TODO: HOW TO USE
* cand apesi pe dpad_up ar trebui sa iti seteze HIGH CONSTRAINT
* cand apesi pe dpad_down ar trebui sa iti seteze LOW CONSTRAINT
* cand apesi pe dpad_right ar trebui sa iti seteze MIDDLE_CONSTRAINT
 */

public class ArmConstraintsTuner extends LinearOpMode {
    private WobbleArm wobbleArm;

    @Override
    public void runOpMode() throws InterruptedException {

        wobbleArm = new WobbleArm(hardwareMap);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        try (InputStream input = getClass().getResourceAsStream("/org/firstinspires/ftc/teamcode/drive/constants.properties")) {
            //load a properties file
            Properties constants = new Properties();
            constants.load(input);

            //create an output file
            File outfile = new File(getClass().getResource("/org/firstinspires/ftc/teamcode/drive/constants.properties").getPath());

            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.dpad_up) {
                    int highTicks = wobbleArm.getPosition();

                    //save high constraint
                    constants.setProperty("WA_HIGH_CONSTRAINT", String.valueOf(highTicks));
                    constants.store(new FileOutputStream(outfile), null);
                }
                if (gamepad1.dpad_down) {
                    int lowTicks = wobbleArm.getPosition();

                    //save high constraint
                    constants.setProperty("WA_LOW_CONSTRAINT", String.valueOf(lowTicks));
                    constants.store(new FileOutputStream(outfile), null);
                }
                if (gamepad1.dpad_right) {
                    int middleTicks = wobbleArm.getPosition();

                    //save high constraint
                    constants.setProperty("WA_MIDDLE_CONSTRAINT", String.valueOf(middleTicks));
                    constants.store(new FileOutputStream(outfile), null);
                }
                if(gamepad1.a){
                    wobbleArm.setSpeed(-200);
                    //so it spins up
                    sleep(500);
                    while (wobbleArm.getSpeed() < -190){
                        idle();
                    }
                    wobbleArm.setSpeed(0);
                    int lowTicks = wobbleArm.getPosition() + 300; //TODO de TWEAKUIT VALOAREA ASTA (daca ajunge cumva sa testati modu asta)
                    int middleTicks = lowTicks + 465;
                    int highTicks = lowTicks + 930;

                    constants.setProperty("WA_LOW_CONSTRAINT", String.valueOf(lowTicks));
                    constants.setProperty("WA_MIDDLE_CONSTRAINT", String.valueOf(middleTicks));
                    constants.setProperty("WA_HIGH_CONSTRAINT", String.valueOf(highTicks));
                    constants.store(new FileOutputStream(outfile), null);
                }
            }
        }
        catch (IOException e){
            e.printStackTrace();
        }
    }
}
