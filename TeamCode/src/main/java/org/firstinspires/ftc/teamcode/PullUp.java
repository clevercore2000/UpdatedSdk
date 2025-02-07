package org.firstinspires.ftc.teamcode;

public class PullUp {

    public Hardware hw;
    public ToggleButton toggleServo;
    public ArmDev pullServo;

    public PullUp(Hardware hardware){
        hw = hardware;
        toggleServo = new ToggleButton();
        pullServo = new ArmDev(hw.pullServo, 100);
        pullServo.setRange(ConfigVar.PullUp.PULL_MIN,ConfigVar.PullUp.PULL_MAX);
    }


    public void motorControl(boolean button1, boolean button2){
        if(button2) {
            hw.pullMotor.setPower(ConfigVar.PullUp.motorPower);
            return;
        }
        if(button1)
        {
            hw.pullMotor.setPower(-ConfigVar.PullUp.motorPower);
            return;
        }
        hw.pullMotor.setPower(0);
    }

    public void servoControl(boolean button){
        if(toggleServo.Toggle(button)){
            pullServo.moveTo(ConfigVar.PullUp.servoIdle);
        }
        else {
            pullServo.moveTo(ConfigVar.PullUp.servoPullPos);
        }
    }









}
