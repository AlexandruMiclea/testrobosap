package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class PrototipAruncare {
    private DcMotor motorAruncare;
    private Servo servoBrat;

    public PrototipAruncare(HardwareMap hardwareMap){
        motorAruncare = hardwareMap.dcMotor.get("motor");

        motorAruncare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorAruncare.setDirection(DcMotorSimple.Direction.FORWARD);
        motorAruncare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void rotate( double speed){
        speed = Range.clip(speed, -0.9, 0.9);
        motorAruncare.setPower(speed);
    }

    public void stop(){
        motorAruncare.setPower(0);
    }
}
