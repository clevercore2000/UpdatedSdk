package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class EMAFilter
{

private double prevInput;
private double actInput;
ElapsedTime tm = new ElapsedTime();
double filter( double inputValue, double emaCoef )
{
    actInput = emaCoef* inputValue + ( 1 - emaCoef ) * prevInput;
    prevInput = actInput;
    return actInput;
}

}
