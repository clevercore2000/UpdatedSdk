package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    * This class implements a class for all Arm systems of the robot
    * The object would move and estimate completion of move using the elapsed time using the length of travel and the user defined speed (defSpeed)
    * This class defines the common attributes for all Servo driven systems of the robot:
    ** tgPos, actPos - are the target and actual position as a value defining the position of the servo {0 .. 1}
    **  defSpeed - the define speed ( which is constant for servo ). This speed is how fast servo moves and consider also the load applied
    ** tm timer - is used to determine the elapsed time between the calls of the method execute()
    ** The method called  setTgPos( newPosition) has to be called to initiate a move
    ** In JogOpMode or AutoOpMode :
        - call setTgPos( newPosition )
        - call execute() method in the Op loop
*/

/*
Salut Matei,
Am observat in codul pt sisteme ca toate comenzile sunt indentice si ce se schimba e ce comanda se da si cui se da comanda.
Defapt comanda de exemplu  "GripperClosed" este folosita ca sa se apeleze functia
hardware.gripper.setPosition(ConfigVar.ArmCfg.gripperClosedPos);
cu parametru gripperClosedPos care este o valoare Double.

Astfel, de ce sa nu apelam direct setPosition cu parametrul Double??

Am creat o clasa ArmDev care contine atributele unui servo:
Servo servo - este un obiect de tip Servo din clasa Hardware.
ServoState  - o sa actualizam cu starea servoului ( stationar = servoReady sau miscare = servoMove)
trgPos, prevPos - sunt pozitiile tagrget si anterioara care sunt folosite pt calculul distantei parcurse dx = (trg - prev ) de care avem nevoie sa calculam durata deplasarii
defSpeed - este o valoare constanta a vitezei pe care o setam pt fiecare Servo ( cat de repede se deplaseaza gripper, handler, ... ) e nevoie tot pt calcul durata deplasare
tm  - timer pt determinarea duratei deplasarii :) necasara tot pentru calcul durata deplasare

ArmDev  - constructor clasa cu parametrii : Servo si defSpeed

Metode:
setPosition( double newPosition)
    - initiaza o deplasare la pozitia newPosition
    - pozitia va fi de exemplu ConfigVar.ArmCfg.gripperOpenPos care este o valoare fixata in Config.java
    - trece starea servo-ului in "servoMove"
    - initiaza timerul .. defapt il reseteaza cu tm.reset() Time=0

execute()
    - Aceasta metoda asteapta ca starea servo-ului sa fie "seroMove" si atunci cand durata deplasarii e mai lunga de t = dx/defSpeed trece servo-ul in stare "servoReady"
    - Cand se ajunge in servoReady inseamna ca servo-ul si-a incheiat deplasarea - este nevoie aceasta stare in AutoOpMode ca sa stim cand trecem la urmatorul pas din prg autonom

Cum folosim clasa:
In MainOpMode sau AutoOpMode declaram obiecte servo de care avem nevoie. De exemplu:
ArmDev  gripperArm;
ArmDev  handlerArm;
ArmDev  transferArm;
ArmDev  poleArm;
ArmDev  turnerArm;

Apoi le instantiem in RunOpMode(). Astfel avem
armGripper  = new ArmDev( hardware.gripper);
armHandler  = new ArmDev( hardware.handler);
armPole     = new ArmDev( hardware.pole);

Apoi in loop-ul ( while(true) ) din RunOpMode() scriem cum se misca. De exemplu.

if( gamepad2.touchpad_finger_1 ) armGripper.setPosition( ConfigVar.ArmCfg.gripperOpenPos );
if( gamepad2.touchpad_finger_2 ) armGripper.setPosition( ConfigVar.ArmCfg.gripperClosePos );

if( gamepad2.touchpad_finger_1_x ) armPole.setPosition( ConfigVar.ArmCfg.poleUpPos );
if( gamepad2.touchpad_finger_2_x ) armPole.setPosition( ConfigVar.ArmCfg.poleIdlePos );
... samd
si tot in MainOpMode e nevoie sa apelam si execute()
armGripper.execute();
*/

public class ArmDev
{
    Servo servo;    // Servo hardware
    enum  ServoState {servoReady, servoMove}; // Status of ArmDev object
    private String gIndex;  // Index number used in the R-Code parser
    //    ConfigVar configVar;
    public ServoState state = ServoState.servoReady;    // Current status of the servo
    public double actPos;       // Actual position with range Min .. Max
    public double trgPos;       // Target position ( set-point position)
    private double prevPos;      // prev position
    private double   defSpeed;   // The defined fixed speed of the arm
    private double  minRange;
    private double  maxRange;
    private final double IN_WINDOW = 2;
    ElapsedTime tm;             // Timer
    // Method: ArmDev
    // Constructor
    ArmDev( Servo armServo, String rcGIndex,double definedSpeed)
    {
        servo = armServo;

        gIndex = rcGIndex;
        defSpeed = definedSpeed;
        tm = new ElapsedTime();
        tm.startTime();
    }
    // Method: ArmDev
    // Constructor
    ArmDev( Servo armServo, double definedSpeed )
    {
        servo = armServo;
        gIndex = "";
        defSpeed = definedSpeed;
        tm = new ElapsedTime();
        tm.startTime();
    }
    // Method: setRange
    // Sets the range of servo arm in degree
    public void setRange( double in_minRange, double in_maxRange )
    {
        minRange = in_minRange;
        maxRange = in_maxRange;
    }
    // Method: mapRange
    // Returns position in servo units (0 .. 1)
    private double mapRange(double inValue)
    {
        return ( ((inValue - minRange) * /*servo_full_range*/1) / (maxRange - minRange) + /*servo_min*/0 );
    }
    // Returns actual servo position in ArmDev range units minRange ... maxRange
    double getActPos()
    {
        return ( (servo.getPosition()*( maxRange-minRange )) / /*(in_max-in_min)*/ + minRange );
    }

    // Method: moveTo
    // This method starts the move and sets appropriate status to arm status
    public void moveTo(double newPosition)
    {
        servo.setPosition( mapRange( newPosition ) );   // Initiates the move to required position
        trgPos = newPosition;               // Save the target position - necessary for calculation of travel duration
        state = ServoState.servoMove;       // Sets the arm in moving state
        tm.reset();                         // Reset the timer dt = 0;
    }
    // Method: moveTo
    // This method starts the move and sets appropriate status to arm status
    public void moveTo(String newPosition)
    {
        if(newPosition == null ) return;
        if(newPosition == "NOP") return;
        double numPosition = Double.parseDouble(newPosition);
        servo.setPosition( mapRange( numPosition ) );   // Initiates the move to required position
        trgPos = numPosition;               // Save the target position - necessary for calculation of travel duration
        state = ServoState.servoMove;       // Sets the arm in moving state
        tm.reset();                         // Reset the timer dt = 0;
    }

    /*
    Method: execute
    ** This function has to be called every machine cycle
    ** Waits for the state to become "servoMode" and for the time to elapse then sets the servo in "servoReady"
    ** The status "servoReady" and "servoMove" are necessary in the AutoOpMode

    */
    public void execute()
    {
        if(state == ServoState.servoMove )
        {
            double dt = tm.milliseconds()/1000; // calculate time elapsed from start of move ( in seconds)
            // We use speed formula v = dx/t to determine the time that elapsed since servo has start moving
            // t = dx/v in our case dx = ( trgPos - actPos) and v = defSpeed
            // The servo would take the time t  in order to complete the move.
            // If it past more than t seconds than set the servo in servoReady state

            if(  dt > ( Math.abs(trgPos-prevPos ))/defSpeed  )
            {
                prevPos = trgPos;
                state = ServoState.servoReady;
            }

        }
    }

    // Method: stop
    // Stops the servo immediately
    public void stop()
    {
        // Set the actual position as target position will stop the moves
        servo.setPosition( servo.getPosition() );
    }
    // Method: getIndex
    // Returns a string with the index number used to identify the servo in the R-Code parser
    public String getIndex(){ return gIndex; }
    // Method: isReady
    // Returns true when status is servoReady
    public boolean isReady()
    {
        return ( state == ServoState.servoReady );
    }
    // Method: notReady is the oposite of isReady
    public boolean notReady()
    {
        return ( state != ServoState.servoReady );
    }

    // Method: inPosition
    // Returns true if the servo is in the chkPosition position
    public boolean inPosition( double chkPosition)
    {
        return ( Math.abs(chkPosition - getActPos() )< IN_WINDOW );
    }
}
