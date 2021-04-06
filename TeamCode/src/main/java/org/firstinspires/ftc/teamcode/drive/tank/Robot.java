package org.firstinspires.ftc.teamcode.drive.tank;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.localization.vision.OpenCVThread;
import org.firstinspires.ftc.teamcode.drive.subsystems.CollectionMechanism;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDriveChassis;
//import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipAruncare;
import org.firstinspires.ftc.teamcode.drive.subsystems.ThrowingMechanism;
import org.firstinspires.ftc.teamcode.drive.subsystems.WobbleArm;
//import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipGarou;
//import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipSistColectare;

public class Robot {
    private boolean initialize;
    public MecanumDriveChassis drive;
    public OpenCVThread openCV;
    public ElapsedTime timer;
    //public WobbleArm wobbleArm;

    public ThrowingMechanism thrower;
    //public CollectionMechanism collector;

    public Robot (HardwareMap hardwareMap) {
        initialize = true;

        drive = new MecanumDriveChassis(hardwareMap);
        openCV = new OpenCVThread(hardwareMap);
        timer = new ElapsedTime();

        //wobbleArm = new WobbleArm(hardwareMap);
        thrower = new ThrowingMechanism(hardwareMap);
        //collector = new CollectionMechanism(hardwareMap);

        initialize = false;
    }

    public boolean isInitialize() {return initialize;}
}
