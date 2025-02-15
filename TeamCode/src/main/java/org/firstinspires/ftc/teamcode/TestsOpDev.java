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
    private ArmDev poleArm1;
    private ArmDev poleArm2;
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
            poleArm1 = new ArmDev(hardware.poleServo1, 150 );
            poleArm2 = new ArmDev(hardware.poleServo2, 150 );

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

        poleArm1.setRange(ConfigVar.ArmCfg.POLE_MIN1,ConfigVar.ArmCfg.POLE_MAX1);
        poleArm1.moveTo( ConfigVar.ArmCfg.poleIdle);
        poleArm2.setRange(ConfigVar.ArmCfg.POLE_MIN2,ConfigVar.ArmCfg.POLE_MAX2);
        poleArm2.moveTo( ConfigVar.ArmCfg.poleIdle);
        turnerArm.setRange(ConfigVar.ArmCfg.TURNER_MIN,ConfigVar.ArmCfg.TURNER_MAX);
//        turnerArm.moveTo( ConfigVar.ArmCfg.turnerIdle);
    }

    void processAllSystems()
    {
        mecanumDev.execute();
        sliderDev.execute();
        gripperArm.execute();
        handlerArm.execute();
        poleArm1.execute();
        poleArm2.execute();
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
            // mecanumDev.MoveRTo( 100, 0.1);

            // mecanumDev moves manual according to joystick inputs
//            mecanumDev.jogMoveXYR(gamepad1.left_stick_x, gamepad1.left_stick_y,-gamepad1.right_stick_x);
            // Slider moves manual according to joystick inputs when not in pick sample process
            sliderDev.moveJog( -gamepad2.left_stick_y );

        if( gripperToggle.Toggle( gamepad2.dpad_up)  )
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed );
        else
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened );

        if(handleToggle.Toggle(gamepad2.dpad_left))
            handlerArm.moveTo( ConfigVar.ArmCfg.handlerClosed );
        else
            handlerArm.moveTo( ConfigVar.ArmCfg.handlerOpened );

        if( poleToggle.Toggle(gamepad2.dpad_down ) ){
            poleArm1.moveTo( 0);
            poleArm2.moveTo( 0);
            //polePos = ConfigVar.ArmCfg.poleIdle;
        }
        else {
            poleArm1.moveTo(180);
            poleArm2.moveTo(180);
           //polePos = ConfigVar.ArmCfg.poleSpPrePick;
        };

        if( transferToggle.Toggle(gamepad2.circle) )
            transferArm.moveTo(ConfigVar.ArmCfg.transferSpCoop);
        else
            transferArm.moveTo( ConfigVar.ArmCfg.transferSpPreCoop );

        if( turnerToggle.Toggle(gamepad2.triangle) )
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
        else
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerFlipped);
        // Time quanta
        dT = tm.milliseconds();
        tm.reset();
        // Call execution of all robot objects
        processAllSystems();
            //telemetry.addData("PrePick", cagePrePick);
            //telemetry.addData("Pick:", cagePick);
/*
        telemetry.addData("whPos", mecanumDev.odo.actWheelsPos[0]);
        telemetry.addData("whSpe", mecanumDev.odo.actWheelsSpeed[0]);
        telemetry.addData("Pos:" , mecanumDev.odo.actPos[0]);
        telemetry.addData("Spe" , mecanumDev.odo.actSpeed[0]);
        telemetry.addData("sPos:", sliderDev.actPos);
 */
//        telemetry.addData("gripper:", hardware.gripperServo.getPosition());
//        telemetry.addData("sliderSts:", sliderDev.Status);
//        telemetry.addData("trgPos:", sliderDev.targetPos);
//        telemetry.addData("actSpe:", sliderDev.actSpeed);
//        telemetry.addData("spSpe", sliderDev.sspSpeed );
//        telemetry.addData("sPow:", sliderDev.sliderPower);
//        telemetry.addData("spSpeed:", sliderDev.getSpSpeed());
//        telemetry.addData("actPos:", sliderDev.getActPosition());
            telemetry.addData("Mec:", mecanumDev.isReady());
            telemetry.addData("Slid",sliderDev.isReady());
            telemetry.addData("Grip:", gripperArm.isReady());
            telemetry.addData("Hand:", handlerArm.isReady());
            telemetry.addData("Turn:", turnerArm.isReady());
            telemetry.addData("Tran:", transferArm.isReady());


//        telemetry.addData("P-Kp:", ConfigVar.Slider.POS_KP);
//        telemetry.addData("S-Kp:", ConfigVar.Slider.SPEED_KP);
//        telemetry.addData("S-Ki:", ConfigVar.Slider.SPEED_KI);
//        telemetry.addData("S-Ki:", ConfigVar.Slider.SPEED_KD);



            telemetry.update();

        }
    }
}