package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AbsoluteEncoder {
    public AnalogInput encoder;
    public double lastIndexVoltage=0;
    public double turnIndex = 0;
    public double voltageWithIndex =0;

    public static double MAX_VOLTAGE =3.25;

    public double initVolt;

    public void setInitVolt(double initVoltage){
        this.initVolt = initVoltage;
    }

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
}
