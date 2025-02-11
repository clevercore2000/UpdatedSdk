
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;


public class ConfigVar {
    @Config
    public static class Mecanum {
        // The IN_WINDOW constants define the positioning accuracy ( used in inPosition method )
        //  ** targetPosition is in the range [ targetPosition - IN_WINDOW , ... , targetPosition + IN_WINDOW ]

        public static double IN_WINDOW_X = 50;
        public static double IN_WINDOW_Y = 50;
        public static double IN_WINDOW_R = 50;

        // Joystick EXPO factor {0,..,1}
        public static double STICK_EXPO = 0.0D;         // This will make a curved response of the stick input
        public static double STICK_DEAD_ZONE = 0.0D;    // A dead-zone to help the driver improve accuracy of control with joysticks
        public static double JOG_P_TRAVEL = 1000.0D;    // Positioning travel inJOG ( this should be > than the diagonal of the playing field XY )
        public static double JOG_R_TRAVEL = 361.0D;     // Positioning rotation in JOG ( > 360 deg)
        public static double JOG_P_SPEED = 1.0D;        // Positioning speed in JOG
        public static double JOG_R_SPEED = 1.0D;        // Rotation speed in JOG
        public static double SPEED_GAIN = 1.0D;         // Positioning speed gain factor in the PID controller
        public static double MOTOR_POWER_GAIN = 1.0D;   // Given by gain factor between robot speed [m/s] and corresponding wheels speed [ticks/sec]
    }
    @Config
    public static class Slider{
        // Joystick EXPO factor {0,..,1}
        public static double STICK_EXPO = 0.0D;
        // Position and Speed PI-Controller parameters
        public static double POS_KP = 10.0D;    // Position PID - proportional coefficient
        public static double SPEED_KP = 0.001D;      // Speed PID - proportional coefficient
        public static double SPEED_KI = 0.00001D;     // Speed PID - Integrator coefficient
        public static double SPEED_KD = 0.00001D;       // Speed PID - Derivative coefficient
        public static double IN_WINDOW = 50;
        public static double MAX_TRAVEL = 4000;     // old robot had slider extended to max 2100 ticks and travelled it in 1.5 sec
        public static double HOLD_SPEED = 30;       // Maximum speed
        //public static double MAX_ACCEL = 500;
        public static double MAX_POWER = 1.0D;
        //public static double MAX_SPEED = 1700; // Encoder-ticks/sec
        public static double EMA_FILTER = 0.6;
        // Speed Ramp Generator
        //  * uses logistic function to generate a setpoint signal for the speed controller
        public static double STICK_DEAD_ZONE = 0.0D;
        public static double STICK_GAIN =  1.0D;  // Joystick input value
        public static double JOG_SPEED = 2000.0D;
        public static double MAX_HEIGHT = 3900.0D;
        public static double MIN_HEIGHT = 100.0D;
        public static double SP_PRE_PICK = 1200.0D; //
        public static double SP_PICK = 600.0D;      //
        public static double SP_PLACE = 2100.0D;
        public static double SA_HOME = 500.0D;
        public static double SA_PRE_PICK = 450;

        // Predefined positions ( would this even work??)
        /*
         * A strategy is to move the sliders to predefined positions then the driver would control the robot manually just for the local movements
         */
        public static double HOME_POS = 0;    // Home position ( fully retracted ?? )
        public static double LOW_BASKET_POS = 1500; // Low basket position
        public static double TOP_BASCKET_POS = 2100;  // Top basket position
        public static double GROUND_PICKUP_POS = 50;  // Ground pickup position
    }
    @Config
    public static class Odometry{
        public static double ENCODER_TICKS_PER_REV = 384.5;
        public static double WHEEL_DIAM = 104;
        public static double WHEEL_SEPARATION_WIDTH = 413.7;
        public static double WHEEL_SEPARATION_LENGTH = 336;
    }
    @Config
    public static class ArmCfg{
        public static double GRIPPER_MIN = 0;
        public static double GRIPPER_MAX = 180;
        public static double GRIPPER_SPEED = 100;
        public static double gripperOpened = 125;
        public static double gripperClosed= 11;

        public static double HANDLER_MIN = -90;
        public static double HANDLER_MAX = +90;
        public static double HANDLER_SPEED = 100;
        public static double handlerOpened = 25;
        public static double handlerClosed = 5;

        public static double TRANSFER_MIN = 0;
        public static double TRANSFER_MAX = 1800;
        public static double TRANSFER_SPEED = 100;
        public static double transferSpPreCoop = 0;
        public static double transferSpCoop = 600;

        public static double POLE_MIN1 = 0;
        public static double POLE_MAX1 = 180;
        public static double POLE_MIN2 = 0;
        public static double POLE_MAX2  = 180;
        public static double POLE_SPEED = 100;
        public static double poleSpPick = 20;
        public static double poleSpPrePick = 60;
        public static double poleSpPlace = 0;
        public static double poleSaPrePick = 120;
        public static double poleSaPick = 150;
        public static double poleHome = 65;
        public static double poleIdle = 20;
        public static double poleSaPlace = 70;

        public static double TURNER_MIN = 0;
        public static double TURNER_MAX = 1800;
        public static double TURNER_SPEED = 100;
        public static double turnerIdle = 1000;
        public static double turnerFlipped = 340;
    }

    @Config
    public static class PullUp{

        public static double motorPower = 0.7;

        public static double PULL_MIN = 0;
        public static double PULL_MAX = 180;
        public static double PULL_SPEED = 100;
        public static double servoIdle = 45;
        public static double servoPullPos = 25;

    }
}