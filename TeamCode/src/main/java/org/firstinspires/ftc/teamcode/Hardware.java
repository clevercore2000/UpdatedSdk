package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware
{
    public ConfigVar configVar;
    public  DcMotor rightFront;
    public   DcMotor leftFront ;
    public DcMotor rightBack;
    public DcMotor leftBack ;
    public  DcMotor sliderMotor1 ;
    public  DcMotor sliderMotor2;

    public IMU imu;


    public Servo gripperServo;
    public Servo handlerServo;
    public Servo transferServo;
    public Servo poleServo;
    public Servo turnerServo;

    public ColorSensor colorSensor;
    public Hardware( HardwareMap hw ) {

        configVar = new ConfigVar();

        /**DcMotors**/
        //Chassis
        rightFront = hw.get(DcMotor.class, "rightFront");
        leftFront = hw.get(DcMotor.class, "leftFront");
        rightBack = hw.get(DcMotor.class, "rightBack");
        leftBack = hw.get(DcMotor.class, "leftBack");
        //Lift(Slider)
        sliderMotor1 = hw.get(DcMotor.class, "sliderMotor1");
        sliderMotor2 = hw.get(DcMotor.class, "sliderMotor2");

        //Telling motor what to do when no power (BRAKE or FLOAT)
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sliderMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**Setting up the encoders **/
        //Reseting to 0
        sliderMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Kinda counter-intuitive, makes the motor run at whatever velocity is achieved by apply a particular power level to the motor
        sliderMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Reversing the motors that are head to head
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        //**********************************************************************************//
        /**Servos**/
        gripperServo = hw.get(Servo.class, "Gripper");
        poleServo = hw.get(Servo.class, "Slider");
        //poleServo.setDirection(Servo.Direction.REVERSE);
        handlerServo = hw.get(Servo.class, "Handler");
        transferServo = hw.get(Servo.class, "Transfer");
        turnerServo = hw.get(Servo.class, "Turner");
        /**Sensor**/
        colorSensor = hw.get(ColorSensor.class, "colorSensor");
    }
}