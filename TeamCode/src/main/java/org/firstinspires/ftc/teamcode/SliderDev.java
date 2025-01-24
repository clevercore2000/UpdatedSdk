package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.controller.PIDController;
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
    public double IN_WINDOW = ConfigVar.Slider.IN_WINDOW;
    public double MAX_TRAVEL = ConfigVar.Slider.MAX_TRAVEL;// old robot had slider extended to max 2100 ticks and travelled it in 1.5 sec
    public double MAX_SPEED = ConfigVar.Slider.MAX_SPEED;
    public double MAX_POWER = ConfigVar.Slider.MAX_POWER;
    public  double MAX_HEIGHT= ConfigVar.Slider.MAX_HEIGHT;
    public  double MIN_HEIGHT= ConfigVar.Slider.MIN_HEIGHT;
    // Speed Ramp Generator
    //  * uses logistic function to generate a setpoint signal for the speed controller
    public double MAX_SPEED_LF = ConfigVar.Slider.MAX_SPEED_LF;  // Speed Gain coefficient in Logistic Function (LF) (?? test appropriate value ??)
    public double RATE_SPEED_LF = ConfigVar.Slider.RATE_SPEED_LF;  // Change Rate value in Logistic function f(x) = sspGainCoef/( 1+e^(-sspLogRate*speedSetPoint)
    public double DMP_LPF = ConfigVar.Slider.DMP_LPF;  // Dumping factor used in LowPassFilter ramp generator. Value range is ( 0..1 ) where 0 means no dumping
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

    public enum SliderStatus {SliderReady, SilderMoveJog, SliderMoveAuto, SliderFinished}
    public SliderStatus Status = SliderStatus.SliderReady;

    private double actPos = 0;           // Actual position
    private double prevPos = 0;          // Preview position
    private double actSpeed = 0;         // Actual Speed
    private double targetPos = 0.0D;     // Target position
    private double targetSpeed = 0.0D;   // Target speed
    private double maxSpeed;             // Maximum speed of slider
    private double prevTrgSpeed=0;      // Previous target speed
    private PIDController sliderSpeedController;    // PID controller to drive the speed of the slider
    private double slider1Power;         // Power slider 1
    private double slider2Power;         // Power slider 2
    private final ElapsedTime timer = new ElapsedTime();    // Timer
    private double dT;                  // Time quanta between execute() method calls
    // Method: SliderDev
    // Constructor
    public SliderDev(Hardware hw)
    {
        hardware = hw;
        configVar = new ConfigVar();
    }

    /*
     *   Method: speedRampGenerator
     *   ** Implements a smoothen Acceleration/Deceleration Ramp using a 1st order dumping filter
     *   ** out = kf*in_1 + ( 1 - Kf)*in_0 where Kf={0,..1} is the dumping factor ,in_0 - actual input value, in_1 - previous input value
     *   ** The Acc/Dec is triggered by the change of the targetSpeed value
     *
     */
    private double speedRampGenLPF( double inputTargetSpeed )
    {
        double retunTarget = DMP_LPF*prevTrgSpeed + (1-DMP_LPF)*inputTargetSpeed;
        prevTrgSpeed = retunTarget;
        return retunTarget;
    }
    // Method: speedRampGenLF
    // Generates a S shaped ramp for the speed set-point using Logistic function
    private double speedRampGenLF(double inputTargetSpeed)
    {
        // Return the Logistic function applied to speedTarget
        // This will be a S shape ramp value depending on the parameters speedLogRate &  speedLogGain
        return ( inputTargetSpeed * MAX_SPEED_LF)/(1 + Math.exp( RATE_SPEED_LF * MAX_SPEED ));
    }


    // Method: Limitter ** Returns a value of inputValue but that do not exceeds (-maxValue, +maxValue)
    private double  Limitter( double inputValue, double maxValue )
    {
        return ( Math.max(-maxValue, Math.min(inputValue, maxValue)));
    }
    // Method: Initialize ** Initializes Slider parameters
    public void Initialize()
    {
        // Set speed controller PID parameters - this call should be in an initialisation function - it is required to execute once when robot is powered up
        sliderSpeedController = new PIDController(SPEED_KP, SPEED_KI, SPEED_KD);
        prevTrgSpeed=0;
        timer.startTime();
        timer.reset();
    }

     // Method:execute ** This function has to be called every machine cycle
     /*
     *    ** Implements a motion control for the Sliders 1 & 2
     *   ** Slider1 is master, slider2 follows slider1 ( Matei's concept )
     *   ** For the speed control it generates a S shape acceleration/deceleration ramp
     *   ** It controls the Sliders using a PID that drives the speed and uses position to trigger the move and direction of Sliders
     *   ** Finally the control values are applied to motors power
     *
     *   *** It may require to implement power control limits [minPower .. maxPower] to avoid overstress the motors ( ... these limites could already be implemted in the DCMotor class )
     *       This limits are required because the PID controller would require "infinite" power from motors in certain conditions
     */
    public void execute()
    {
        /*
         *  ** tagetPosition and target Speed are set in the MoveTo and ManualMove method
         */

        // Read the elapsed time from last timer.reset() call
        // actual value of Time - dT is what timer counted since last Timer.reset() call
        dT = timer.milliseconds(); // Timer interval between two consecutive app scans
        timer.reset();

        // Calculate the actual speed of the slider v = ( X-Xo )/(T-To)
        actPos = hardware.sliderMotor1.getCurrentPosition();
        actSpeed = (actPos - prevPos/*it is actually previous position*/) / dT;
        // Read actual position of the Slider
        actPos = hardware.sliderMotor1.getCurrentPosition();   // now we update with actual position
        prevPos = actPos;
        if( Status == SliderStatus.SliderMoveAuto )
        {
            // Calculates the position deviation as (targetPos - actPos) and the targetSpeed output of P-Controller (posDeviation, posKp )
            // Target speed is limitted to range of (-MAX_SPEED, +MAX_SPEED)
            //TODO Varinanta cu limitter - vezi sensul +/- // targetSpeed = Limitter(((targetPos - actPos) * POS_KP), MAX_SPEED);
            targetSpeed = ((targetPos - actPos) * POS_KP);

            if( inPosition() )
            {
                targetSpeed = 0;
                Status = SliderStatus.SliderReady;
            }
            // Smothen the taget speed setpoint for the PI controller
            //targetSpeed = speedRampGenLPF(targetSpeed);
            //targetSpeed /= MAX_SPEED;
        }
        if( Status == SliderStatus.SilderMoveJog )
        {
            targetSpeed = speedRampGenLPF(targetSpeed);
        }

        maxSpeed = Math.max(targetSpeed, maxSpeed);
        // Computes PI-Controller DCMotor Power for Slider for a control speed deviation = ( targetSpeed - actSliderSpeed )
        // Power applied to DCMotor of the slider
        //TODO Varianta cu Limmiter - vezi sensul deplasarii // slider1Power = Limitter(sliderSpeedController.calculate( /*actSliderSpeed*/hardware.sliderMotor1.getPower(), /*setpoinPower*/targetSpeed), MAX_POWER);
        slider1Power = sliderSpeedController.calculate( /*actSliderSpeed*/hardware.sliderMotor1.getPower(), /*setpoinPower*/targetSpeed);
        // Apply to Slider2 the speed setpoint of the slider1 ( ... requires testing to check if stable )
        //  if not stable then it will be required to add P-controller to slider1 as well
        slider2Power = sliderSpeedController.calculate( /*actSliderSpeed*/hardware.sliderMotor1.getPower(), /*setpoinPower*/targetSpeed);
        // Apply calculated control value to Slider
        hardware.sliderMotor1.setPower(slider1Power);
        hardware.sliderMotor2.setPower(slider2Power);
        updateConfig();
    }

    //   Method: stop ** Stop all moves and puts the DCMotors in Freewheel
    public void stop()
    {
        // ??? Do we need to implement safety ???
        // Until above question is answered just stop movements
        targetPos = actPos;
    }


     //   Method: MoveTo ** Request to move the sliders to specified position
    public void moveTo( double trgPos, double trgSpeed)
    {
        targetPos = Math.min(MAX_TRAVEL, trgPos );
        targetSpeed = trgSpeed;
    }
    // Method: moveTo **  This method starts the move with JOG_SPEED
    public void moveTo( double trgPos )
    {
        targetPos = Math.min(MAX_TRAVEL, trgPos );
        targetSpeed = ConfigVar.Slider.JOG_SPEED;
    }
    // Method: moveTo ** This method receives position and speed as Strings
    public void moveTo( String trgPos, String trgSpeed )
    {
        if(trgPos == null || trgSpeed == null ) return;
        targetPos = (trgPos != "NOP")?Math.min(MAX_TRAVEL, Double.parseDouble(trgPos) ):targetPos;
        targetSpeed = ( trgSpeed != "NOP")? Double.parseDouble(trgSpeed):targetSpeed;
        Status = SliderStatus.SliderMoveAuto;
    }

     //   Method: jogMove ** Moves the slider with Joystick input
     /*  ** Joystick in range {-1 .. 1}
     *   ** Joystick input value is proportional with the desired speed ( Ex: the speed increases/decreases proportionally with the forward/backward travel of the stick )
     */
    public void moveJog( double stickSlider )
    {
        // TODO targetPos = ( stickExpo(stickSlider) < -STICK_DEAD_ZONE )? -MAX_TRAVEL : (stickExpo(stickSlider) > STICK_DEAD_ZONE )? + MAX_TRAVEL : 0;
        targetSpeed = stickSlider * STICK_GAIN;
        // Software limits
        if( stickSlider>0 && actPos >= MAX_HEIGHT ) targetSpeed = 0;
        if( stickSlider<0  && actPos <= MIN_HEIGHT ) targetSpeed = 0;
        Status = SliderStatus.SilderMoveJog;
    }

    // Method: inPosition ** Returns true if slider reached the target position
    public boolean inPosition(){ return (Math.abs( targetPos-actPos ) < IN_WINDOW); }

    // Methods: get__ ** Returns the named parameter
    public double getActPosition(){ return actPos;  }
    public double getActualSpeed(){ return actSpeed; }
    public double getTargetSpeed(){ return targetSpeed; }
    public double getDeviation(){ return ((targetPos - actPos) * POS_KP);}
    // Method: isReady ** Returns true is slider completed all moves
    public boolean isReady(){ return ( Status == SliderStatus.SliderReady); }
    // Method: isReady ** Returns true is slider is not ready
    public boolean notReady(){ return ( Status != SliderStatus.SliderReady); }
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
        MAX_SPEED = ConfigVar.Slider.MAX_SPEED;
        MAX_POWER = ConfigVar.Slider.MAX_POWER;
        MAX_HEIGHT = ConfigVar.Slider.MAX_HEIGHT;
        MIN_HEIGHT = ConfigVar.Slider.MIN_HEIGHT;
        // Speed Ramp Generator
        //  * uses logistic function to generate a setpoint signal for the speed controller
        MAX_SPEED_LF = ConfigVar.Slider.MAX_SPEED_LF;  // Speed Gain coefficient in Logistic Function (LF) (?? test appropriate value ??)
        RATE_SPEED_LF = ConfigVar.Slider.RATE_SPEED_LF;  // Change Rate value in Logistic function f(x) = sspGainCoef/( 1+e^(-sspLogRate*speedSetPoint)
        DMP_LPF = ConfigVar.Slider.DMP_LPF;  // Dumping factor used in LowPassFilter ramp generator. Value range is ( 0..1 ) where 0 means no dumping
        STICK_DEAD_ZONE = ConfigVar.Slider.STICK_DEAD_ZONE;
        STICK_GAIN = ConfigVar.Slider.STICK_GAIN;  // Joystick input value
        JOG_SPEED = ConfigVar.Slider.JOG_SPEED;

        // Predefined positions ( would this even work??)
        /*
         * A strategy is to move the sliders to predefined positions then the driver would control the robot manually just for the local movements
         */
        HOME_POS = ConfigVar.Slider.HOME_POS;    // Home position ( fully retracted ?? )
        LOW_BASKET_POS = ConfigVar.Slider.LOW_BASKET_POS; // Low basket position
        TOP_BASCKET_POS = ConfigVar.Slider.TOP_BASCKET_POS;  // Top basket position
        GROUND_PICKUP_POS = ConfigVar.Slider.GROUND_PICKUP_POS;  // Ground pickup position}
    }
}
