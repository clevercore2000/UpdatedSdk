package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.util.ElapsedTime;

//package org.firstinspires.ftc.teamcode;


public class EncOdoDev
{


    //TODO  private IMU imu;
    public  final double ENCODER_TICKS_PER_REV = ConfigVar.Odometry.ENCODER_TICKS_PER_REV;
    public final double WHEEL_DIAM = ConfigVar.Odometry.WHEEL_DIAM;
    public final double WHEEL_SEPARATION_WIDTH = ConfigVar.Odometry.WHEEL_SEPARATION_WIDTH;
    public final double WHEEL_SEPARATION_LENGTH = ConfigVar.Odometry.WHEEL_SEPARATION_LENGTH;

    private final int tkLFr = 0;    // Index for Left-Front encoder
    private final int tkRFr = 1;    // Index for Right-Front encoder
    private final int tkLBk = 2;    // Index for Left-Back encoder
    private final int tkRBk = 3;    // Index for Right-Back encoder

    private final int X = 0;    // Index for the X axis
    private final int Y = 1;    // Index for the Y axis
    private final int R = 2;    // // Index for the Direction
    public double [] actTicks = {0,0,0,0}; // Actual encoders vector

    public double [] prevTicks = {0,0,0,0}; // Preview encoers vector

    public double [] actWheelsPos = {0,0,0,0}; // Wheels positions vector

    public double [] actWheelsSpeed = {0,0,0,0}; // Actual wheels speed vector

    // Robot position
    public double [] actPos = {0, 0, 0, 0};    // Actual position vector
    public double [] prevPos  = {0, 0, 0, 0};  // Preview position vector
    public double [] actSpeed  = {0, 0, 0, 0}; // Actual speed vector

    Hardware hardware;
    ElapsedTime tm;
    private double dT = 0;

    // Constructor
    EncOdoDev( Hardware hw)
    {
        hardware = hw;
        tm = new ElapsedTime();

    }
    // Method: Initialize
    // Initializes variables of odometry
    public void initialize()
    {
        //TODO imu.resetYaw();
        setHomePos(0, 0 , 0);
        tm.startTime();
    }
    // Method: setHomePos
    // Sets the value for Home ( offset )
    public void setHomePos(double homeX, double homeY, double homeR)
    {
        actPos[X] = homeX;      // actual position X axis (encoder_ticks)
        actPos[Y] = homeY;
        actPos[R] = homeR;   // Angular direction (deg ?? )
    }


    public void computeWheelsTicks()
    {
        // Actual Encoder ticks ( read from encoders )
         actTicks[tkLFr] = hardware.leftFront.getCurrentPosition();
        actTicks[tkRFr] = hardware.rightFront.getCurrentPosition();
        actTicks[tkLBk] = hardware.leftBack.getCurrentPosition();
        actTicks[tkRBk] = hardware.rightBack.getCurrentPosition();
        // Calculate wheels speeds
        actWheelsSpeed[tkLFr] = actTicks[tkLFr] - prevTicks[tkLFr] / dT;
        actWheelsSpeed[tkRFr] = actTicks[tkRFr] - prevTicks[tkRFr] / dT;
        actWheelsSpeed[tkLBk] = actTicks[tkLBk] - prevTicks[tkLBk] / dT;
        actWheelsSpeed[tkRBk] = actTicks[tkRBk] - prevTicks[tkRBk] / dT;
        // Calculate wheels positions
//        actPos[X]   += computeWheelsDistance(actTicks[tkLFr], prevTicks[tkLFr]);
//        actPos[Y]   += computeWheelsDistance(actTicks[tkRFr], prevTicks[tkRFr]);

    }

    /*
     *    Method: computeDistance
     *   ** calculated the distance in encoder Ticks traveled by a wheel
     *   ** Distance = ( actualTicks - PreviousTicks)/( Encoder_Ticks_Per_Rev x 2 x PI )
     *
     */
    private double computeWheelsDistance(double actTicks, double prevTicks)
    {
        return ( ( actTicks - prevTicks ) * WHEEL_DIAM * Math.PI /ENCODER_TICKS_PER_REV );
    }

    /*
     *   Method: computeRobotSpeed
     *   ** From wheel velocities, compute base velocity by using the inverse kinematics
     *
     */

    private void computeSpeeds() {
        actSpeed[X] = ( actWheelsSpeed[tkLFr] + actWheelsSpeed[tkLFr] + actWheelsSpeed[tkLBk] + actWheelsSpeed[tkRBk]) * (WHEEL_DIAM * Math.PI /ENCODER_TICKS_PER_REV) / 4;
        actSpeed[Y] = (-actWheelsSpeed[tkLFr] + actWheelsSpeed[tkRFr] + actWheelsSpeed[tkLBk] - actWheelsSpeed[tkRBk]) * (WHEEL_DIAM * Math.PI /ENCODER_TICKS_PER_REV) / 4;
        actSpeed[R] = (-actWheelsSpeed[tkLFr] + actWheelsSpeed[tkRFr] - actWheelsSpeed[tkLBk] + actWheelsSpeed[tkRBk]) * (WHEEL_DIAM * Math.PI /ENCODER_TICKS_PER_REV) / (4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH));
    }

    /*
     *  Method: computePosition
     *   ** Calculates the actual position of the robot based on actual speed, direction and time interval
     *      * Note that the speeds are calculated using direct reading of wheels encoders
     *   x = v*dt - Integral of the speed applied on interval dt = [prev_Time , act_Time]
     *
     *  ** An alternative is to calculate position [X,Y,R] based on a direct reading of position of mecanum wheels encoders
     *      * supposedly direct reading of the encoders would deliver grater accuracy
     *      * an external dedicate encoder would be better
     *
     */
    private void computePosition()
    {
        // Compute the position in on the ground
        // Home position (x,y) is in the teams's corner
        // We compute the orientation as well
        // - the position is not affected by orientation
        // The robot can also slide left/right which affects the position [X, Y]

        // Update previous position vector
        prevPos[X] = actPos[X];
        prevPos[Y] = actPos[Y];
        prevPos[R] = actPos[R];
        // Calculate actual position vector
        actPos[X] += ( ( (actSpeed[X] *  cos(actSpeed[R]) ) - (actSpeed[Y] *  sin(actSpeed[R]) ) ) * dT );
        actPos[Y] += ( ( (actSpeed[X] *  sin(actSpeed[R]) ) + (actSpeed[Y] *  cos(actSpeed[R]) ) ) * dT );
        actPos[R] += ( actSpeed[R] * dT );
        // Update preview ticks for the next iteration
        prevTicks[tkLFr] = actTicks[tkLFr];
        prevTicks[tkRFr] = actTicks[tkRFr];
        prevTicks[tkLBk] = actTicks[tkLBk];
        prevTicks[tkRBk] = actTicks[tkRBk];
    }

    // Method: execute()
    // This method has to be called every cycle in the MainOpMode and AutoOpMode or once in the MecanumDev object
    public void execute()
    {
        dT = tm.seconds();   // Get a value in seconds
        tm.reset();
        computeWheelsTicks();
        computeSpeeds();
        computePosition();
    }
    // The mothods getAct___() returns a vector with actual values requested
    public double [] getPosVector()   { return actPos;    }
    public double [] getPrevPosVector() { return prevPos; }
    public double [] getSppedVector() { return actSpeed; }
}
