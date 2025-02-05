package org.firstinspires.ftc.teamcode;
public class ToggleButton
{
    boolean toggleState = false;
    boolean riseUp = false;
    boolean prevState = false;
    //ToggleButton()
    boolean reState;

    boolean Toggle(boolean inState)
    {
        reState = false;
        if( inState && ! riseUp )
        {
            toggleState = !toggleState;
            reState = true;
            riseUp = true;
        }
        if( !inState ) riseUp = false;
        return toggleState ;
    }

}
