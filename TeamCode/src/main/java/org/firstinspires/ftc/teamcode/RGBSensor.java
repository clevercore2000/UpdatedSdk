package org.firstinspires.ftc.teamcode;

public class RGBSensor
{
private Hardware hw;
enum ColorValue {redValue, yellowValue, blueValue, blackValue, otherValue};
ColorValue colorValue = ColorValue.blackValue;

RGBSensor(Hardware hardware)
{
    hw = hardware;
}

ColorValue getColorValue()
    {
        int red = hw.colorSensor.red();
        int blue = hw.colorSensor.blue();
        int green = hw.colorSensor.green();
        if( red > 80 && green > 80 && blue < 45 ) return ColorValue.yellowValue;
        if( red > 80 && green > 60 && blue > 40 ) return ColorValue.redValue;
        if( red < 60 && green > 60 && blue > 80 ) return ColorValue.blueValue;
        if( red < 60 && green > 60 && blue < 80 ) return ColorValue.blackValue;
        return ColorValue.otherValue;
    };

}
