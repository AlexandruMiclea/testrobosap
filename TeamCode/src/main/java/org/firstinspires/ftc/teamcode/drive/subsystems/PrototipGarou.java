package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PrototipGarou {
    private CRServo servoGarou;


    public PrototipGarou(HardwareMap hardwareMap) {

        servoGarou = hardwareMap.crservo.get("servoGarou");
        servoGarou.setPower(0.5);
        servoGarou.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void moveForward() { servoGarou.setPower(1); }
    public void moveBackward() { servoGarou.setPower(0); }
    public void stop(){
        servoGarou.setPower(0.5);
    }
}
