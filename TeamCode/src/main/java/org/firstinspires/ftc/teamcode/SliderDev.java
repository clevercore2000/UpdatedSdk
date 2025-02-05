package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

//@SliderDrive
public class SliderDev{
    /*
     *   ** This class should be inherit from a ROBOT class that should implement:
     *   - Robot status ( Initialized / opMode(Manu/Auto) / Connected
     *   - Manage the 30 sec autonomus timing
     *   - The main loop iteration ( or implement with threads ?? )
     *   - Controls
     */
    Hardware hardware;
    ConfigVar configVar;

    // Joystick EXPO factor {0,..,1}
    public double STICK_EXPO = ConfigVar.Slider.STICK_EXPO;
    // Position and Speed PI-Controller parameters
    public double POS_KP = ConfigVar.Slider.POS_KP;  // Position PID - proportional coefficient
    public double SPEED_KP = ConfigVar.Slider.SPEED_KP;    // Speed PID - proportional coefficient
    public double SPEED_KI = ConfigVar.Slider.SPEED_KI;     // Speed PID - Integrator coefficient
    public double SPEED_KD = ConfigVar.Slider.SPEED_KD;     // Speed PID - Derivative coefficient
    public double IN_WINDOW = ConfigVar.Slider.IN_WINDOW;   // Used to check slider is in position actPos = {-IN_WINDOW,..,+IN_WINDOW}
    public double HOLD_SPEED = ConfigVar.Slider.HOLD_SPEED; // Used while slider is in sliderReady to hold the position
    public double MAX_TRAVEL = ConfigVar.Slider.MAX_TRAVEL; // old robot had slider extended to max 2100 ticks and travelled it in 1.5 sec
    public double MAX_POWER = ConfigVar.Slider.MAX_POWER;
    public  double MAX_HEIGHT= ConfigVar.Slider.MAX_HEIGHT;
    public  double MIN_HEIGHT= ConfigVar.Slider.MIN_HEIGHT;
    public double STICK_DEAD_ZONE = ConfigVar.Slider.STICK_DEAD_ZONE;
    public double STICK_GAIN =  ConfigVar.Slider.STICK_GAIN;  // Joystick input value
    public double JOG_SPEED = ConfigVar.Slider.JOG_SPEED;
    // Predefined positions ( would this even work??)
    /*
     * A strategy is to move the sliders to predefined positions then the driver would control the robot manually just for the local movements
     */
    public double HOME_POS = ConfigVar.Slider.HOME_POS;    // Home position ( fully retracted ?? )
    public double LOW_BASKET_POS = ConfigVar.Slider.LOW_BASKET_POS; // Low basket position
    public double TOP_BASCKET_POS = ConfigVar.Slider.TOP_BASCKET_POS;  // Top basket position
    public double GROUND_PICKUP_POS = ConfigVar.Slider.GROUND_PICKUP_POS;  // Ground pickup position

    public enum SliderStatus {sliderReady, sliderMoveJog, sliderMoveAuto }  // Status of the slider
    public SliderStatus Status = SliderStatus.sliderReady;
    private double actPos = HOME_POS;           // Actual position
    private double prevPos = 0;          // Preview position
    public double actSpeed = 0;         // Actual Speed
    private double spSpeed = 0;         // Set-Point speed ( input for the speed control loop)
    public double targetPos = 0.0D;     // Target position
    private double targetSpeed = 0.0D;   // Target speed
    private double maxAccel = 0;
    private PIDControl pidSlider1;    // PID controller to drive the speed of the slider
    public double sliderPower;         // Power slider 1
    public double joystickValue;

    private final ElapsedTime tm = new ElapsedTime();    // Timer
    EMAFilter emaSpeed = new EMAFilter();
    EMAFilter emaPos = new EMAFilter();
    public double dT;                  // Time quanta between execute() method calls
    // Constructor
    public SliderDev(Hardware hw)
    {
        hardware = hw;
        configVar = new ConfigVar();
        pidSlider1 = new PIDControl(SPEED_KP, SPEED_KI,SPEED_KD);
        tm.startTime();
        tm.reset();
        pidSlider1.setMaxOut(1);
    }


     // Method:execute
     /*
        ** This function has to be called every machine cycle
        ** Implements a motion control for the Sliders 1 & 2
        ** Position control is a P ( proportional ) controller that outputs set-point speed spSpeed
        ** When position is reached ( IN_WINDOW ) the targetSpeed is set as HOLD_SPEED. It is design to have enough power to maintain/correct actual position while holding
        ** The set-point speed that is output by P-Control is limited to targetSpeed ( given by the user)
        ** Actual speed is calculated actSpeed = (actPos-prePos)/dT and is filtered with an EMA filter to cancel noise
        ** The motors of the sliders gets power from a PID that is driven by spSpeed
        ** Holding position is insured by setting the targetPos = actPos
        ** Finally the control values are applied to motors power
        ** Slider1 is master, slider2 follows slider1 ( Matei's concept )
     */
    public void execute()
    {
         //  ** tagetPosition and target Speed are set in the MoveTo and ManualMove method
        dT = tm.milliseconds(); // Timer interval between two consecutive app scans
        if( dT < 3.0 /* 3 millisecond sampling time*/ ) return; // Sampling time is 1 millisecond
        dT /= 1000; // Convert to seconds from now on
        tm.reset(); // Restart clock for the next sample tm
        // Read actual position of the Slider
        actPos = hardware.sliderMotor1.getCurrentPosition();
        // Calculate the actual speed of the slider v = ( X-Xo )/(T-To) and filer it with an EMA filter
        actSpeed = emaSpeed.filter( (actPos - prevPos)/dT , ConfigVar.Slider.EMA_FILTER/*EMA-Coeficient*/);
        prevPos = actPos;
        // Speed control depends on Status of the slider
        switch( Status )
        {
            case sliderReady: // While in Ready state it holds current position with HOLD_SPEED
                if( Math.abs(joystickValue) > 0 ){  Status = SliderStatus.sliderMoveJog; }
                spSpeed = limitValue( (( targetPos - actPos ) * POS_KP) , targetSpeed);
                break;
            case sliderMoveJog: // While in JOG state it just passed targetSpeed from Joystick ( well with some gains - see MoveJog method)
                if( Math.abs(joystickValue) > 0 )
                {
                    // When joystick is out of rest we only speed control ( no position control ) ...
                    spSpeed = targetSpeed = joystickValue * STICK_GAIN * JOG_SPEED; // set targetSpeed as JOG_SPEED. Adjust STICK_GAIN if required
                    break;
                }
                // While joystick is in rest , set position as actual position and then HOLD current position
                targetPos = actPos;
                spSpeed = targetSpeed = HOLD_SPEED; // Only control speed
                Status = SliderStatus.sliderReady;
                break;
            case sliderMoveAuto: // While in Auto state it sets the speed according to targetSpeed and position error but goes to Ready when position window is reached
                spSpeed = limitValue( (( targetPos - actPos ) * POS_KP) , targetSpeed);
                if( inPosition() )
                {
                    targetSpeed = HOLD_SPEED;
                    Status = SliderStatus.sliderReady;
                }
                break;
        }
        // Calculate required power to DC Motors using PID Control
        sliderPower = pidSlider1.calculate(spSpeed,actSpeed);
        // Apply calculated control value to Slider
        hardware.sliderMotor1.setPower(sliderPower);
        hardware.sliderMotor2.setPower(sliderPower);
        updateConfig();
    }
    //   Method: stop ** Stop all moves and puts the DCMotors in Freewheel
    public void stop()
    {
        targetPos = actPos;
        targetSpeed = HOLD_SPEED;
        Status = SliderStatus.sliderReady;
    }

    // Method: Limiter ** Returns a value of inputValue but that do not exceeds (-maxValue, +maxValue)
    private double  limitValue( double inputValue, double maxValue )
    {
        return ( Math.max(-maxValue, Math.min(inputValue, maxValue)));
    }
    // Method: Limiter ** Returns a value of inputValue but that do not exceeds (-maxValue, +maxValue)
    private double  limitValue( double inputValue, double minValue, double maxValue )
    {
        return ( Math.max(minValue, Math.min(inputValue, maxValue)));
    }
    //   Method: MoveTo ** Request to move the sliders to specified position
    public void moveTo( double trgPos, double trgSpeed)
    {
        targetPos = limitValue( trgPos, MIN_HEIGHT, MAX_HEIGHT );
        targetSpeed = trgSpeed;
        Status = SliderStatus.sliderMoveAuto;
    }
    // Method: moveTo **  This method starts the move with JOG_SPEED
    public void moveTo( double trgPos  )
    {
        targetPos = limitValue( trgPos, MIN_HEIGHT, MAX_HEIGHT );
        targetSpeed = ConfigVar.Slider.JOG_SPEED;
        Status = SliderStatus.sliderMoveAuto;
    }
    // Method: moveTo ** This method receives position and speed as Strings
    public void moveTo( String trgPos, String trgSpeed )
    {
        if(trgPos == null || trgSpeed == null ) return;
        targetPos = (trgPos != "NOP")?limitValue( Double.parseDouble(trgPos), 0, MAX_TRAVEL ):targetPos;
        targetSpeed = ( trgSpeed != "NOP")? Double.parseDouble(trgSpeed):targetSpeed;
        Status = SliderStatus.sliderMoveAuto;
    }
     //   Method: jogMove ** Moves the slider with Joystick input
     /*  ** Joystick in range {-1 .. 1}
     *   ** Joystick input value is proportional with the desired speed ( Ex: the speed increases/decreases proportionally with the forward/backward travel of the stick )
     *   ** While in sliderModeAuto ( like when executing a sequence) the jog mode is disabled
     *   ** While joystick input is ZERO the targetSpeed is set to HOLD_SPEED
     */
    public void moveJog( double stickSlider )
    {
        // TODO targetPos = ( stickExpo(stickSlider) < -STICK_DEAD_ZONE )? -MAX_TRAVEL : (stickExpo(stickSlider) > STICK_DEAD_ZONE )? + MAX_TRAVEL : 0;
        joystickValue = ( Math.abs(stickSlider) > STICK_DEAD_ZONE)? stickSlider : 0;
    }
    // Method: inPosition ** Returns true if slider reached the target position
    public boolean inPosition(){ return (Math.abs( targetPos-actPos ) < IN_WINDOW); }

    // Methods: get__ ** Returns the named parameter
    public double getActPosition(){ return actPos;  }
    public double getActSpeed(){ return actSpeed; }
    public double getSpSpeed(){return spSpeed; }
    public double getTargetSpeed(){ return targetSpeed; }
    public double getDeviation(){ return ((targetPos - actPos) * POS_KP);}
    // Method: isReady ** Returns true is slider completed all moves
    public boolean isReady(){ return ( Status == SliderStatus.sliderReady); }
    // Method: isReady ** Returns true is slider is not ready
    public boolean notReady(){ return ( Status != SliderStatus.sliderReady); }
    // Method: stickExpo : Adds EXPO to stick input
    private double stickExpo(double stickIn ){ return ( stickIn * ( 1 - STICK_EXPO ) + STICK_EXPO * Math.pow(stickIn,3) );    }
    // Method: updateConfig ** updates the variables from cConfigVar
    // The method is used during debug - it allows user to modify values of variables
    public void updateConfig() {
        STICK_EXPO = ConfigVar.Slider.STICK_EXPO;
        // Position and Speed PI-Controller parameters
        POS_KP = ConfigVar.Slider.POS_KP;  // Position PID - proportional coefficient
        SPEED_KP = ConfigVar.Slider.SPEED_KP;    // Speed PID - proportional coefficient
        SPEED_KI = ConfigVar.Slider.SPEED_KI;     // Speed PID - Integrator coefficient
        SPEED_KD = ConfigVar.Slider.SPEED_KD;     // Speed PID - Derivative coefficient
        IN_WINDOW = ConfigVar.Slider.IN_WINDOW;
        MAX_TRAVEL = ConfigVar.Slider.MAX_TRAVEL;// old robot had slider extended to max 2100 ticks and travelled it in 1.5 sec
        MAX_POWER = ConfigVar.Slider.MAX_POWER;
        MAX_HEIGHT = ConfigVar.Slider.MAX_HEIGHT;
        MIN_HEIGHT = ConfigVar.Slider.MIN_HEIGHT;
        STICK_DEAD_ZONE = ConfigVar.Slider.STICK_DEAD_ZONE;
        STICK_GAIN = ConfigVar.Slider.STICK_GAIN;  // Joystick input value
        JOG_SPEED = ConfigVar.Slider.JOG_SPEED;
        HOLD_SPEED = ConfigVar.Slider.HOLD_SPEED;
        HOME_POS = ConfigVar.Slider.HOME_POS;    // Home position ( fully retracted ?? )
        LOW_BASKET_POS = ConfigVar.Slider.LOW_BASKET_POS; // Low basket position
        TOP_BASCKET_POS = ConfigVar.Slider.TOP_BASCKET_POS;  // Top basket position
        GROUND_PICKUP_POS = ConfigVar.Slider.GROUND_PICKUP_POS;  // Ground pickup position}
    }
}
