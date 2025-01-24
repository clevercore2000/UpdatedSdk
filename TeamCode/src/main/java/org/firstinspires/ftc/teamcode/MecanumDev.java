package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *   class MecanumControl
 *   ** Implements a mecanum drive controlled manually or autonomus
 *   ** The atributs and methods that are declared PUBLIC are the user/programmer interface
 *   ** Methods Move__To have distance (in encoder_ticks ) and speed as encoder_ticks/sec
 *   ** Moves using joystickare not implemented yet - we just need to decide how joystick inputs are used ( not a difficult job anyway :) )
 */

public class MecanumDev// extends Thread
{

    /*
     *   ** This class should be inherit from a ROBOT class that should implement:
     *   - Robot status ( Initialized / opMode(Manu/Auto) / Connected
     *   - Manage the 30 sec autonomus timing
     *   - The main loop iteration ( or implement with threads ?? )
     *   - Controls connection
     *   - Telemetry connection
     */

    /*
     *   run
     *   ** This is the "main" method of the thread
     *   ** The thread needs to be started from outside the class ( from an opMode may be??? )
     *
     */
    public Hardware hardware;
    public enum MecanumStatus {MecanumReady, MecanumMoveJog, MecanumMoveAuto, MecanumFinished}
    private MecanumStatus Status = MecanumStatus.MecanumReady;
    public MecanumStatus getStatus() { return Status; }


    // Motors are removed from hardware
    //static HardwareMap hw;

    private EncOdoDev odo;
    ConfigVar configVar;
    // Retrieve the IMU from the hardware map

    // The IN_WINDOW constants define the positioning accuracy ( used in inPosition method )
    //  ** targetPosition is in the range [ targetPosition - IN_WINDOW , ... , targetPosition + IN_WINDOW ]
    public final double IN_WINDOW_X = ConfigVar.Mecanum.IN_WINDOW_X;
    public final double IN_WINDOW_Y = ConfigVar.Mecanum.IN_WINDOW_Y;
    public final double IN_WINDOW_R = ConfigVar.Mecanum.IN_WINDOW_R;

    private final int X = 0;    // X axis index in vectors
    private final int Y = 1;    // Y axis index in vectors
    private final int R = 2;    // Direction index in vectors
    // Joystick EXPO factor {0,..,1}
    public final double STICK_EXPO = ConfigVar.Mecanum.STICK_EXPO;         // This will make a curved response of the stick input
    private final double STICK_DEAD_ZONE = ConfigVar.Mecanum.STICK_DEAD_ZONE;   // A dead-zone to help the driver improve accuracy of control with joysticks
    private final double JOG_P_TRAVEL = ConfigVar.Mecanum.JOG_P_TRAVEL;     // Positioning travel inJOG ( this should be > than the diagonal of the playing field XY )
    private final double JOG_R_TRAVEL = ConfigVar.Mecanum.JOG_R_TRAVEL;     // Positioning rotation in JOG ( > 360 deg)
    private final double JOG_P_SPEED = ConfigVar.Mecanum.JOG_P_SPEED;      // Positioning speed in JOG
    private final double JOG_R_SPEED = ConfigVar.Mecanum.JOG_R_SPEED;       // Rotation speed in JOG
    private final double SPEED_GAIN = ConfigVar.Mecanum.SPEED_GAIN;       // Positioning speed gain factor in the PID controller
    public double MOTOR_POWER_GAIN = ConfigVar.Mecanum.MOTOR_POWER_GAIN; // Given by gain factor between robot speed [m/s] and corresponding wheels speed [ticks/sec]
    // Robot position/speed
    private double [] actPos = {0, 0, 0};
    private double [] errPos = {0, 0, 0};       // Position deviation X,Y,R for mecanum moves
    private double [] targetPos = {0, 0, 0};    // target position on X,Y,R
    public double [] actSpeed = {0, 0, 0};     // actual speed X,Y,R
    private double actSpeedXY = 0;    // Actual speed alond the path in XY space
    private double targetSpeedXY = 0;    // Target speed alond the path in XY space
    private double targetSpeedR = 0;    // Target speed alond the path in XY space
    private final ElapsedTime tm = new ElapsedTime();
    private double dT;
    // Method: MecanumDev ** Constructor
    public MecanumDev(Hardware hw)
    {
        this.hardware = hw;
        odo = new EncOdoDev(hardware);
        dT = 0;
        configVar  = new ConfigVar();
    }
    // Method: Initialize ** Initializes MecanumDev parameters
    public void Initialize()
    {
        hardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // ** SET the correct direction of motors
        // If not required just comment or remove the bellow lines
        hardware.rightFront.setDirection(DcMotor.Direction.REVERSE);
        hardware.leftFront.setDirection(DcMotor.Direction.FORWARD);
        hardware.rightBack.setDirection(DcMotor.Direction.REVERSE);
        hardware.leftBack.setDirection(DcMotor.Direction.FORWARD);

        // *** Let us reset the IMU that is used for orientation
        // REV Hub's orientation is assumed to be logo up / USB forward
        // May be a good idea to add a dedicate button on driver's controller
        // to allow the option to reset the IMU
        odo.initialize();
        tm.startTime();
    }
    /*
     *   // Method: inPosition
     *   ** Returns TRUE when the deviation of the target position is in the range [ targetPosition - IN_WINDOW/2 , ... , targetPosition + IN_WINDOW/2 ]
     */
    public boolean inPosition()
    {
        return ( (Math.abs(errPos[X]) <= IN_WINDOW_X ) && (Math.abs(errPos[Y]) <= IN_WINDOW_Y) && (Math.abs(errPos[R]) <= IN_WINDOW_R ));
    }
    /*
     *   DriveRobot
     *   ** Drives the robot based on input values
     *   ** Input values can be joystick value (+/-) as direction and value speed
     *   ** Input values can also be commands in Autonomus ( but they are not target positions instead just move which way + which direction)
     *   ** Autonomus moves need to be implemented in another function
     */
    private void MecanumDrive(double speedX, double speedY, double speedR)
    {
        // Calculate mecanum drive power for each motor ( ** calculation algorithm found on internet )
        double maxSpeed = Math.max( Math.abs(speedY) + Math.abs(speedX) + Math.abs(speedR), 1);
        double powerLFr = (speedY + speedX + speedR) / maxSpeed;
        double powerLBk = (speedY - speedX + speedR) / maxSpeed;
        double powerRF = (speedY - speedX - speedR) / maxSpeed;
        double powerRBk = (speedY + speedX - speedR) / maxSpeed;

        // Move the Mecanum motors by applying power
            hardware.leftFront.setPower(powerLFr * MOTOR_POWER_GAIN);
            hardware.leftBack.setPower(powerLBk * MOTOR_POWER_GAIN);
            hardware.rightFront.setPower(powerRF * MOTOR_POWER_GAIN);
            hardware.rightBack.setPower(powerRBk * MOTOR_POWER_GAIN);
    }
    /*
     *   executeMoves
     *   ** This function has to be called evry cycle
     *   ** Executes programmed moves
     *   ** The moves are triggered by Move___To functions
     *
     */
    public void execute()
    {
//        dT = timer.milliseconds() / 1e3D;   // Get a value in seconds
//        timer.reset();
        // Calculates the duration in sec from last iteration
        // If Status is still "Move" then execute a new iteration of the positioning
        odo.execute();

        if (Status == MecanumStatus.MecanumMoveAuto )
        {
            // Reads actual position from the encoder
            actPos = odo.getPosVector();
            // Calculates deviation (position error) on X, Y, R
            errPos[X] = targetPos[X] - actPos[X];
            errPos[Y] = targetPos[Y] - actPos[Y];
            errPos[R] = targetPos[R] - actPos[R];

            // Calculate speeds along direction vectors X & Y
            // it does the interpolation, so that X and Y arrive in the target position in the same time.
            actSpeed[X] = SPEED_GAIN * actSpeedXY * (errPos[X] / sqrt(errPos[X] * errPos[X] + errPos[Y] * errPos[Y]));
            actSpeed[Y] = SPEED_GAIN * actSpeedXY * (errPos[Y] / sqrt(errPos[X] * errPos[X] + errPos[Y] * errPos[Y]));

            MecanumDrive(actSpeed[X], actSpeed[Y], errPos[R]);
            // If all axis reached target position - stop moving
            if (inPosition()) {
                //   This code can helps to remove unnecessary gitter or oscilation if position is acceptably close to target
                targetPos[X] = actPos[X];
                targetPos[Y] = actPos[Y];
                targetPos[R] = actPos[R];

                Status = MecanumStatus.MecanumReady;
            }

        }
        if (Status == MecanumStatus.MecanumMoveJog )
        {
            MecanumDrive(actSpeed[X], actSpeed[Y], actSpeed[R]);
        }
    }
    /*
     *   emergencyStop
     *   ** Stop all moves and puts the DCMotors in Freewheel
     *
     */
    public void stop()
    {
        // ??? Do we need to implement safety ???
        // Until above question is answered just stop movements
        targetPos[X]= actPos[X];
        targetPos[Y] = actPos[Y];
        targetPos[R] = actPos[R];
    }
    /*
     *  Move___To functions
     *  MoveTo
     *  MoveXYTo
     *  MoveXTo
     *  MoveYTo
     *  MoveRTo
     *
     *   ** These functions is dedicated to drive the robot in autonomus or to target position
     *   ** Moves the robot to target positions ( targetX,targetY, targetR ) with specific speed along XY path inputSpeed
     *   ** Rotate speed is fixed - if required we can add a target speed as well
     *   ** Input values ar target position X, Y, R and the speed along the path
     *   ** The actual move is implemented in function executeMove
     *   ** A move can be request using this functions only if robot has completed the previous move or it is switched in manual mode
     *
     */
    public boolean moveTo(double targetX, double targetY, double targetR, double targetSpe)
    {
        if( Status != MecanumStatus.MecanumReady ) return false;
        // Calculate deviation
        targetPos[X] = targetX;
        targetPos[Y] = targetY;
        targetPos[R] = targetR;
        targetSpeedXY = targetSpe;

        Status = MecanumStatus.MecanumMoveAuto;
        return true;
    }
    public boolean moveTo(String targetX, String targetY, String targetR, String targetSpe)
    {
        if( Status != MecanumStatus.MecanumReady ) return false;
        int x =0;
        // If input targets are not "NOP" : No Operation
        if( targetX != "NOP" && targetX != null )
        {
            targetPos[X] = Double.parseDouble(targetX);
        }
        if( targetY != "NOP" && targetY != null ) {
            targetPos[Y] = Double.parseDouble(targetY);

        }
        if( targetR !="NOP" && targetR != null) {
            targetPos[R] = Double.parseDouble(targetR);
        }
        if( targetSpe != "NOP" && targetY != null) {
            targetSpeedXY = Double.parseDouble(targetSpe);
        }


        Status = MecanumStatus.MecanumMoveAuto;
        return true;
    }
    public boolean MoveRTo( double targetR, double targetSpeed)
    {
        if( Status != MecanumStatus.MecanumReady ) return false;
        // Set the target position
        targetPos[R] = targetR;
        targetSpeedR = targetSpeed;
        // Initiate the move
        Status = MecanumStatus.MecanumMoveAuto;
        return true;
    }

    public boolean jogMoveXYR(double stickX, double stickY, double stickR)
    {
        double [] jogDir = {0, 0 ,0 };
        //if( Status != MecanumStatus.MecanumReady ) return false;
        // Determines direction of X,Y,R as { -1 / 0 / 1 }
        jogDir[X] = ( stickExpo(stickX) > STICK_DEAD_ZONE )? 1 : ( stickExpo(stickX) < -STICK_DEAD_ZONE )? -1 : 0;
        jogDir[Y] = ( stickExpo(stickY) > STICK_DEAD_ZONE )? 1 : ( stickExpo(stickY) < -STICK_DEAD_ZONE )? -1 : 0;
        jogDir[R] = ( stickExpo(stickR) > STICK_DEAD_ZONE )? 1 : ( stickExpo(stickR) < -STICK_DEAD_ZONE )? -1 : 0;
        // Speed gain R = absolute of the Joystick value
        double stickSpeed = Math.sqrt( stickX*stickX + stickY*stickY )/2; // Jog speed along the path {-1, .. 1}

        targetSpeedXY = JOG_P_SPEED * stickSpeed; // target speed is percentage of JOG_SPEED given by stick displacemets

        actSpeed[X] = jogDir[X] * Math.abs( stickX );
        actSpeed[Y] = jogDir[Y] * Math.abs( stickY );
        actSpeed[R] = jogDir[R] * Math.abs( stickR );
        Status = MecanumStatus.MecanumMoveJog;
        return true;
    }
    // The mothods getAct___() returns a vector with actual values requested
    public double [] getPosVector()   { return actPos;    } // Returns {actPos[X], actPos[Y], actPos[R]}
    // Actual speeds
    public double [] getSppedVector() { return actSpeed; }  // Returns {actSpeed[X], actSpeed[Y], actSpeed[R]}
    // Actual deviation
    public double [] getDeviationVector()   { return errPos; }  // Returns {errPos[X], errPos[Y], errPos[R]}
    // Moves completed
    public boolean isReady()
    {
        return ( Status == MecanumStatus.MecanumReady );
    }
    private double stickExpo(double stickIn )
    {
        return ( stickIn * ( 1 - STICK_EXPO ) + STICK_EXPO * Math.pow(stickIn,3) );
    }


}




