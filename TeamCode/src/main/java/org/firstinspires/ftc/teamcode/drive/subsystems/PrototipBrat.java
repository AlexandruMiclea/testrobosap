package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PrototipBrat {
    private DcMotor brat;
    public PrototipBrat(HardwareMap hardwareMap){
        brat = hardwareMap.dcMotor.get("motorBratPivotant");
        brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brat.setDirection(DcMotorSimple.Direction.FORWARD);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void moveForward(){
        brat.setPower(0.5);
    }

    public void moveBackward(){
        brat.setPower(-0.5);
    }
    public void stop(){
        brat.setPower(0);
    }

}
