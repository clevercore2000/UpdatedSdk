package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class TestsOpDev extends LinearOpMode
{
    private Hardware hardware;
    // Declare all devices of the Robot
    private ArmDev gripperArm;
    private ArmDev handlerArm;
    private ArmDev poleArm;
    private ArmDev transferArm;
    private ArmDev turnerArm;
    private SliderDev sliderDev;
    private MecanumDev mecanumDev;
    // Elaps timer
    private ElapsedTime tm;
    private double dT=0;
    RCodeParser Parser;

    ToggleButton gripperToggle;
    ToggleButton handleToggle;
    ToggleButton transferToggle;
    ToggleButton turnerToggle;
    ToggleButton poleToggle;
    double polePos;
    public void initialize()
    {
        // ... initialization logic ...
        try {
            hardware = new Hardware(hardwareMap);

            sliderDev = new SliderDev(hardware);
            mecanumDev = new MecanumDev(hardware);

            gripperArm = new ArmDev( hardware.gripperServo, 100);
            handlerArm = new ArmDev(hardware.handlerServo, 100);
            poleArm = new ArmDev(hardware.poleServo, 150 );
            transferArm = new ArmDev(hardware.transferServo, 100);
            turnerArm = new ArmDev(hardware.turnerServo, 300);

            gripperToggle = new ToggleButton();
            handleToggle = new ToggleButton();
            transferToggle = new ToggleButton();
            poleToggle = new ToggleButton();
            turnerToggle = new ToggleButton();

            tm = new ElapsedTime();
            tm.startTime();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public void setHomePositions()
    {
        // .. Set Home/Idle positions of all systems ...
        mecanumDev.Initialize();
        // sliderDev.Status = SliderDev.SliderStatus.SilderMoveJog;

        gripperArm.setRange(ConfigVar.ArmCfg.GRIPPER_MIN, ConfigVar.ArmCfg.GRIPPER_MAX);
//    gripperArm.moveTo( ConfigVar.ArmCfg.gripperOpened);
        handlerArm.setRange(ConfigVar.ArmCfg.HANDLER_MIN, ConfigVar.ArmCfg.HANDLER_MAX);
//    handlerArm.moveTo( ConfigVar.ArmCfg.handlerClosed);

        transferArm.setRange(ConfigVar.ArmCfg.TRANSFER_MIN, ConfigVar.ArmCfg.TRANSFER_MAX);
        transferArm.moveTo( ConfigVar.ArmCfg.transferSpPreCoop);

        poleArm.setRange(ConfigVar.ArmCfg.POLE_MIN,ConfigVar.ArmCfg.POLE_MAX);
        poleArm.moveTo( ConfigVar.ArmCfg.poleIdle);
        turnerArm.setRange(ConfigVar.ArmCfg.TURNER_MIN,ConfigVar.ArmCfg.TURNER_MAX);
//        turnerArm.moveTo( ConfigVar.ArmCfg.turnerIdle);
    }

    void processAllSystems()
    {
        mecanumDev.execute();
        sliderDev.execute();
        gripperArm.execute();
        handlerArm.execute();
        poleArm.execute();
        transferArm.execute();
        turnerArm.execute();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        // ... OpMode logic ...
        // Instantiate all required vars
        this.initialize();

        waitForStart();
        // Set Home/Idle position on all systems
        setHomePositions();
        while (opModeIsActive())
        {
            // mecanumDev moves manual according to joystick inputs
            mecanumDev.jogMoveXYR(gamepad1.left_stick_x, gamepad1.left_stick_y,-gamepad1.right_stick_x);
            // Slider moves manual according to joystick inputs when not in pick sample process
            //sliderDev.moveJog( -gamepad2.left_stick_y );

            gripperToggle.Toggle(gamepad2.dpad_up);
            if(  gripperToggle.reState )
                sliderDev.moveTo( 1500 );
            else
                sliderDev.moveTo( 750 );
            // Use joystick only for tests/debugging the systems
/*
        if(  gripperToggle.Toggle(gamepad2.dpad_up) )
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed );
        else
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened );
*/

        handleToggle.Toggle(gamepad2.dpad_left);
        if(handleToggle.reState)
            handlerArm.moveTo( ConfigVar.ArmCfg.handlerClosed );
        else
            handlerArm.moveTo( ConfigVar.ArmCfg.handlerOpened );

        poleToggle.Toggle(gamepad2.dpad_down );
        if( poleToggle.reState ){
            poleArm.moveTo( ConfigVar.ArmCfg.poleIdle);
            polePos = ConfigVar.ArmCfg.poleIdle;
        }
        else {
            poleArm.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            polePos = ConfigVar.ArmCfg.poleSaPrePick;
        };

        transferToggle.Toggle(gamepad2.circle);
        if( transferToggle.reState )
            transferArm.moveTo(ConfigVar.ArmCfg.transferSpCoop);
        else
            transferArm.moveTo( ConfigVar.ArmCfg.transferSpPreCoop );

        turnerToggle.Toggle(gamepad2.triangle);
        if( turnerToggle.reState )
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
        else
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerFlipped);
        // Time quanta
        dT = tm.milliseconds();
        tm.reset();
        // Call execution of all robot objects
        processAllSystems();

        // Telemetry
        int red = hardware.colorSensor.red();
        int blue = hardware.colorSensor.blue();
        int green = hardware.colorSensor.green();
/*
        telemetry.addData("red", hardware.colorSensor.red());
        telemetry.addData("green", hardware.colorSensor.green());
        telemetry.addData("blue", hardware.colorSensor.blue());
*/
        telemetry.addData("sliderState:", sliderDev.isReady());
        telemetry.addData("actPos", sliderDev.getActPosition());
        telemetry.addData("trgPos:", sliderDev.targetPos);
        telemetry.addData("Pole", polePos );

        // telemetry.addData("transf.:", transferArm.isReady());
        // telemetry.addData("turner:", turnerArm.isReady());
        // telemetry.addData("handler:", handlerArm.isReady());
  /*
        telemetry.addData("transf.:", transferArm.isReady());
        telemetry.addData("turner:", turnerArm.isReady());
        telemetry.addData("handler:", handlerArm.isReady());
        telemetry.addData("gripper:", gripperArm.isReady());
        telemetry.addData("pole:", poleArm.isReady());
        telemetry.addData("sPos:", sliderDev.getActPosition());
        telemetry.addData("sInPos:", sliderDev.inPosition());
*/

            telemetry.update();

        }
    }
}