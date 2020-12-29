package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PrototipGarou {
    private CRServo servoGarou;
    private Servo servoBlocker;
    private DcMotor motorSlider;

    public PrototipGarou(HardwareMap hardwareMap) {
        servoGarou = hardwareMap.crservo.get("servoGarou");
        servoGarou.setPower(0);
        servoGarou.setDirection(DcMotorSimple.Direction.FORWARD);

        motorSlider = hardwareMap.dcMotor.get("motorSlider");
        motorSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlider.setPower(0);
        motorSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoBlocker = hardwareMap.servo.get("servoBlocker");
        servoBlocker.setPosition(0);
    }

    public void expandTube() { servoGarou.setPower(1); }
    public void shrinkTube() { servoGarou.setPower(-1); }
    public void stopTube(){
        servoGarou.setPower(0);
    }

    public void block(){
        servoBlocker.setPosition(1);
    }
    public void unblock(){
        servoBlocker.setPosition(0.3);
    }

    public void slideUp(){
        motorSlider.setPower(0.3);
    }
    public void slideDown(){
        motorSlider.setPower(-0.3);
    }
    public void slideStop(){
        motorSlider.setPower(0);
    }
}
