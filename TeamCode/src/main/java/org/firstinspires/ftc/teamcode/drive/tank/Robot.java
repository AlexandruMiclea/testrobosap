package org.firstinspires.ftc.teamcode.drive.tank;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.localizer.vision.OpenCVThread;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDriveChassis;
//import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipAruncare;
import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipBrat;
//import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipGarou;
//import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipSistColectare;

public class Robot {
    private boolean initialize;
    //public MecanumDriveMR drive;
    public MecanumDriveChassis drive;
    public OpenCVThread openCV;
    public ElapsedTime timer;
    //public PrototipSistColectare sistColectare = null;
    public PrototipBrat bratPivotant;
    //public PrototipGarou bratGarou;
//    public PrototipAruncare protoAruncare;

    public Robot (HardwareMap hardwareMap) {
        initialize = true;

        drive = new MecanumDriveChassis(hardwareMap);
        openCV = new OpenCVThread(hardwareMap);
        timer = new ElapsedTime();
        //sistColectare = new PrototipSistColectare(hardwareMap);
        bratPivotant = new PrototipBrat(hardwareMap);
        //bratGarou = new PrototipGarou(hardwareMap);
//        protoAruncare = new PrototipAruncare(hardwareMap);

        initialize = false;
    }

    public boolean isInitialize() {return initialize;}
}
