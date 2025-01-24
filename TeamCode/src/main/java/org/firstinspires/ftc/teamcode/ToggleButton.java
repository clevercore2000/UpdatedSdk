package org.firstinspires.ftc.teamcode;
public class ToggleButton
{
    boolean toggleState = false;
    boolean riseUp = false;
    boolean prevState = false;

    //ToggleButton()
    boolean Toggle(boolean inState)
    {
        if( inState && ! riseUp )
        {
            toggleState = !toggleState;
            riseUp = true;
        }
        if( !inState ) riseUp = false;
        return toggleState ;
    }

}
