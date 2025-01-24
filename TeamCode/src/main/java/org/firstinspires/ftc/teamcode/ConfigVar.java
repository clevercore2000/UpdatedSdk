
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigVar {
    public static class Mecanum {

        // The IN_WINDOW constants define the positioning accuracy ( used in inPosition method )
        //  ** targetPosition is in the range [ targetPosition - IN_WINDOW , ... , targetPosition + IN_WINDOW ]
        public static double IN_WINDOW_X = 50;
        public static double IN_WINDOW_Y = 50;
        public static double IN_WINDOW_R = 50;

        // Joystick EXPO factor {0,..,1}
        public static double STICK_EXPO = 0.0D;         // This will make a curved response of the stick input
        public static double STICK_DEAD_ZONE = 0.0D;   // A dead-zone to help the driver improve accuracy of control with joysticks
        public static double JOG_P_TRAVEL = 1000.0D;     // Positioning travel inJOG ( this should be > than the diagonal of the playing field XY )
        public static double JOG_R_TRAVEL = 361.0D;     // Positioning rotation in JOG ( > 360 deg)
        public static double JOG_P_SPEED = 1.0D;      // Positioning speed in JOG
        public static double JOG_R_SPEED = 1.0D;       // Rotation speed in JOG
        public static double SPEED_GAIN = 1.0D;       // Positioning speed gain factor in the PID controller
        public static double MOTOR_POWER_GAIN = 1.0D; // Given by gain factor between robot speed [m/s] and corresponding wheels speed [ticks/sec]
    }
    //    @Config
    public static class Slider{

        public static double autoPosition = 100;
        public static double autoSpeed = 2;
        // Joystick EXPO factor {0,..,1}
        public static double STICK_EXPO = 0.0D;
        // Position and Speed PI-Controller parameters
        public static double POS_KP = 1.0D;  // Position PID - proportional coefficient
        public static double SPEED_KP = 0.2D;    // Speed PID - proportional coefficient
        public static double SPEED_KI = 0.0;     // Speed PID - Integrator coefficient
        public static double SPEED_KD = 0.0;     // Speed PID - Derivative coefficient
        public static double IN_WINDOW = 25;
        public static double MAX_TRAVEL = 5000;// old robot had slider extended to max 2100 ticks and travelled it in 1.5 sec
        public static double MAX_SPEED = 100;
        public static double MAX_POWER = 1.0D;
        // Speed Ramp Generator
        //  * uses logistic function to generate a setpoint signal for the speed controller
        public static double MAX_SPEED_LF = 1500.0D;  // Speed Gain coefficient in Logistic Function (LF) (?? test appropriate value ??)
        public static double RATE_SPEED_LF = 12.5D;  // Change Rate value in Logistic function f(x) = sspGainCoef/( 1+e^(-sspLogRate*speedSetPoint)
        public static double DMP_LPF = 0.0D;  // Dumping factor used in LowPassFilter ramp generator. Value range is ( 0..1 ) where 0 means no dumping
        public static double STICK_DEAD_ZONE = 0.0D;
        public static double STICK_GAIN =  5.0D;  // Joystick input value
        public static double JOG_SPEED = 100.0D;
        public static double MAX_HEIGHT = 2700.0D;
        public static double MIN_HEIGHT = 100.0D;
        public static double SP_PRE_PICK = 1500.0D;  // Initial position when pick a new sample with handle
        public static double SP_PICK = 800.0D;

        // Predefined positions ( would this even work??)
        /*
         * A strategy is to move the sliders to predefined positions then the driver would control the robot manually just for the local movements
         */
        public static double HOME_POS = 0;    // Home position ( fully retracted ?? )
        public static double LOW_BASKET_POS = 1500; // Low basket position
        public static double TOP_BASCKET_POS = 2100;  // Top basket position
        public static double GROUND_PICKUP_POS = 50;  // Ground pickup position
    }
    //    @Config
    public static class Odometry{
        public static double ENCODER_TICKS_PER_REV = 8192;
        public static double WHEEL_RADIUS = 104;
        public static double WHEEL_SEPARATION_WIDTH = 413.7;
        public static double WHEEL_SEPARATION_LENGTH = 336;
    }
    //   @Config
    public static class ArmCfg{
        public static double GRIPPER_MIN = 0;
        public static double GRIPPER_MAX = 300;
        public static double gripperOpened = 35;
        public static double gripperClosed= 50;

        public static double HANDLER_MIN = -90;
        public static double HANDLER_MAX = +90;
        public static double handlerOpened = 0.5;
        public static double handlerClosed = 0.5;

        public static double TRANSFER_MIN = -90;
        public static double TRANSFER_MAX = +90;
        public static double transferSpPreCoop = 0.5;
        public static double transferSpCoop = 0.5;


        public static double POLE_MIN = 0;
        public static double POLE_MAX = 300;
        public static double poleSpPick = 20;
        public static double poleSpPlace = 0.5;
        public static double poleSaPrePick = 130;
        public static double poleSaPick = 110;
        public static double poleHome = 220;

        public static double TURNER_MIN = -90;
        public static double TURNER_MAX = +90;
        public static double turnerIdle = 0.5;
        public static double turnerFlipped = 0.5;

        /*
        public static double preScoopPos= 0.5;
        public static double scoopPos = 0.5;
         */

    }
}