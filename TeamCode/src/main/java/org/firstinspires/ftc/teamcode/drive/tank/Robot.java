package org.firstinspires.ftc.teamcode.drive.tank;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDriveMR;
import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipBrat;
import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipGarou;
import org.firstinspires.ftc.teamcode.drive.subsystems.PrototipSistColectare;

public class Robot {
    private boolean initialize;
    //public MecanumDriveMR drive;
    //public PrototipSistColectare sistColectare = null;
    //public PrototipBrat bratPivotant;
    public PrototipGarou bratGarou;

    public Robot (HardwareMap hardwareMap) {
        initialize = true;

        //drive = new MecanumDriveMR(hardwareMap);
        //sistColectare = new PrototipSistColectare(hardwareMap);
       // bratPivotant = new PrototipBrat(hardwareMap);
        bratGarou = new PrototipGarou(hardwareMap);

        initialize = false;
    }

    public boolean isInitialize() {return initialize;}
}
