package org.firstinspires.ftc.teamcode.drive.tank;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipSistColectare;

public class Robot {
    private boolean initialize;
    //public MecanumDrive drive;
    public PrototipSistColectare sistColectare = null;

    public Robot (HardwareMap hardwareMap) {
        initialize = true;

        //drive = new MecanumDrive(hardwareMap);
        sistColectare = new PrototipSistColectare(hardwareMap);

        initialize = false;
    }

    public boolean isInitialize() {return initialize;}
}
