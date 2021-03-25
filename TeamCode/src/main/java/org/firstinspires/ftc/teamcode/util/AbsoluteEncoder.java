package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AbsoluteEncoder {
    private AnalogInput encoder;
    private double lastIndexVoltage=0;
    private double turnIndex = 0;
    private double voltageWithIndex =0;

    private static double MAX_VOLTAGE =3.25;

    private double initVolt;

    public void setInitVolt(double initVoltage){
        this.initVolt = initVoltage;
    }

    public double getMaxVoltage() { return MAX_VOLTAGE; }


    public double getVoltageWithIndex(){
        double current1 = encoder.getVoltage();

        double derivative1 = current1 - lastIndexVoltage;

        if ((derivative1 < -MAX_VOLTAGE / 2) && (derivative1 != 0)){
            turnIndex ++;
        } else if ((derivative1 > MAX_VOLTAGE / 2) && (derivative1 != 0)){
            turnIndex --;
        }

        lastIndexVoltage = current1;

        voltageWithIndex = current1 + (turnIndex * MAX_VOLTAGE) - initVolt;

        return voltageWithIndex;
    }

    public double getTurnIndex(){
        return turnIndex;
    }

    public AbsoluteEncoder(AnalogInput encoder) {
        this.encoder = encoder;
        this.initVolt = encoder.getVoltage();
    }
}
