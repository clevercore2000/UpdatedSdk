package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class MainOpDev extends LinearOpMode {
private Hardware hardware;
// Declare all devices of the Robot
private ArmDev gripperArm;
private ArmDev handlerArm;
private ArmDev poleArm;
private ArmDev transferArm;
private ArmDev turnerArm;
private SliderDev sliderDev;
private MecanumDev mecanumDev;
private PullUp pullUp;
// Elaps timer
private ElapsedTime tm;
private double dT;
ToggleButton gripperToggle;
ToggleButton handleToggle;
ToggleButton transferToggle;
ToggleButton turnerToggle;
ToggleButton poleToggle;
ToggleButton moveToSlider;
RGBSensor rgbSensor;

public void initialize()
{
    // ... initialization logic ...
    try {
        hardware = new Hardware(hardwareMap);
        sliderDev = new SliderDev(hardware);
        mecanumDev = new MecanumDev(hardware);
        pullUp = new PullUp(hardware);
        tm = new ElapsedTime();
        gripperArm = new ArmDev( hardware.gripperServo, ConfigVar.ArmCfg.GRIPPER_SPEED);
        handlerArm = new ArmDev(hardware.handlerServo, ConfigVar.ArmCfg.HANDLER_SPEED);
        poleArm = new ArmDev(hardware.poleServo, ConfigVar.ArmCfg.POLE_SPEED );
        transferArm = new ArmDev(hardware.transferServo, ConfigVar.ArmCfg.TRANSFER_SPEED);
        turnerArm = new ArmDev(hardware.turnerServo, ConfigVar.ArmCfg.TURNER_SPEED);

        gripperToggle = new ToggleButton();
        handleToggle = new ToggleButton();
        transferToggle = new ToggleButton();
        poleToggle = new ToggleButton();
        turnerToggle = new ToggleButton();
        moveToSlider = new ToggleButton();

        rgbSensor = new RGBSensor( hardware );
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
    turnerArm.moveTo( ConfigVar.ArmCfg.turnerIdle);
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

enum CagePrePick { cageIdle, cagePick1,cagePick2, cagePick3, cagePick4, cagePick5, cagePick6, cagePick7 };
CagePrePick cagePrePick = CagePrePick.cageIdle;
public void prePickSACage(){
    switch ( cagePrePick ) {
        case cageIdle:
            if (!gamepad2.square || sliderDev.notReady() || gripperArm.notReady()  || poleArm.notReady()) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            sliderDev.moveTo(ConfigVar.Slider.SA_HOME);
            cagePrePick = CagePrePick.cagePick1;
            break;
        case cagePick1:
            cagePrePick = CagePrePick.cagePick2;
        case cagePick2:
            if (sliderDev.notReady() || gripperArm.notReady()  || poleArm.notReady()) break;
            poleArm.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            cagePrePick = CagePrePick.cagePick3;
            break;
        case cagePick3:
            if (sliderDev.notReady() || gripperArm.notReady()  || poleArm.notReady()) break;
            sliderDev.moveTo(ConfigVar.Slider.SA_PRE_PICK);
            cagePrePick = CagePrePick.cagePick4;
        case cagePick4:
            if (poleArm.notReady() || gripperArm.notReady() || sliderDev.notReady()) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
            cagePrePick = CagePrePick.cagePick5;
            break;
        case cagePick5:
            if ( !gamepad2.square || poleArm.notReady() || gripperArm.notReady() ) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            //poleArm.moveTo(ConfigVar.ArmCfg.poleHome);
            cagePrePick = CagePrePick.cagePick6;
            break;
        case cagePick6:
            if (poleArm.notReady() || gripperArm.notReady() || sliderDev.notReady()) break;
            sliderDev.moveTo(3800);
            poleArm.moveTo(ConfigVar.ArmCfg.poleSaPlace);
            cagePrePick = CagePrePick.cageIdle;
            break;
    }
}

enum CagePick { cagePickIdle, cageGripper, cagePick,cagePrePick}
CagePick cagePick = CagePick.cagePickIdle;
public void pickSACage()
{
    switch ( cagePick ) {
        case cagePickIdle:
            // Check all servo's are ready
            if (!gamepad2.circle || gripperArm.notReady() || poleArm.notReady()) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
            cagePick = CagePick.cageGripper;
            break;
        case cageGripper:
            if (gripperArm.notReady() || poleArm.notReady()) break;
            poleArm.moveTo(ConfigVar.ArmCfg.poleSaPick);
            cagePick = CagePick.cagePick;
        case cagePick:
            // Check all servo's are ready
            if (gripperArm.notReady() || poleArm.notReady()) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            // Set status for next step in sequence
            cagePick = CagePick.cagePrePick;
            break;
        case cagePrePick:
            if (gripperArm.notReady() || poleArm.notReady()) break;
            poleArm.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            cagePick = CagePick.cagePickIdle;
            break;
    }
}

//enum PickUSample {pickIdle, pickStart, pickFlip, pickTransfer, pickPreGrab,pickGrab, pickSliderUp, pickPolePlace, pickToIdle }
    enum PickSample { pickIdle, pickSP1 ,pickSP2, pickSP3, pickSP4, pickSP5, pickSP6, pickSP7, pickSP71, pickSP8, pickSP9, pickSP10 }
PickSample pickUpSample = PickSample.pickIdle;

/*
This method implements the sequence to use SliderArm, handlerArm, transferArm, turnerArm and colorSensor
to detect flip if needed, transfer and grab the sample
*/
public void pickSpecimen()
{
    switch( pickUpSample )
    {
        case pickIdle:
            // Check all servo's are ready
            if( !gamepad2.triangle || sliderDev.notReady() || handlerArm.notReady() || transferArm.notReady() || turnerArm.notReady() ) break;
            sliderDev.moveTo( ConfigVar.Slider.SA_HOME);
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerClosed);
            pickUpSample = PickSample.pickSP1;
            break;
        case pickSP1:
            if( poleArm.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            poleArm.moveTo(ConfigVar.ArmCfg.poleSpPrePick);
            pickUpSample = PickSample.pickSP2;
            break;
        case pickSP2:
            if( poleArm.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            sliderDev.moveTo(ConfigVar.Slider.SP_PRE_PICK ); // Move slider to pre-pick position ( retracted up)
            pickUpSample = PickSample.pickSP3;
            break;
            case pickSP3:
            if( poleArm.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            //pickUpSample = ( rgbSensor.getColorValue() != RGBSensor.ColorValue.blackValue )? PickSample.pickSP4 : PickSample.pickSP5;
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerOpened);
            pickUpSample = PickSample.pickSP5;
            break;
        case pickSP4: // Flip Turner
            if( poleArm.notReady() || transferArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerFlipped); // Flip the sample
            // Set status for next step in sequence
            pickUpSample = PickSample.pickSP5;
            break;
        case pickSP5:
            if( poleArm.notReady() || transferArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            transferArm.moveTo(ConfigVar.ArmCfg.transferSpCoop);
            // Set status for next step in sequence
            pickUpSample = PickSample.pickSP6;
            break;
        case pickSP6:
            if( transferArm.notReady() ) break;
            sliderDev.moveTo(ConfigVar.Slider.SP_PICK );
            // We can also move the Turner back in Idle position
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
            // Set status for next step in sequence
            pickUpSample = PickSample.pickSP7;
            break;
        case pickSP7:
            if( sliderDev.notReady() || turnerArm.notReady() ) break; // Wait slider to complete it's move
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerClosed);
            // Set status for next step in sequence
            pickUpSample = PickSample.pickSP71;
            break;
        case pickSP71:
            transferArm.moveTo(ConfigVar.ArmCfg.transferSpPreCoop);
            pickUpSample = PickSample.pickSP8;
            break;
        case pickSP8:
            if( sliderDev.notReady() || handlerArm.notReady()) break; // Wait slider to complete it's move
            sliderDev.moveTo(ConfigVar.Slider.SP_PLACE );
            pickUpSample = PickSample.pickSP9;
            break;
        case pickSP9:
            if( !gamepad2.dpad_up && sliderDev.notReady() ) break; // Wait slider to complete it's move
            poleArm.moveTo(ConfigVar.ArmCfg.poleSpPlace );
            pickUpSample = PickSample.pickSP10;
        case pickSP10:
            if( poleArm.notReady() ) break; // Wait for handler to grab the sample
            // Return the transferArm to pre-coop position
            transferArm.moveTo(ConfigVar.ArmCfg.transferSpPreCoop);
            // All pickSample operation completed - go back to Idle
            pickUpSample = PickSample.pickIdle;
            break;
    }
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

    while ( opModeIsActive() )
    {
        // mecanumDev moves manual according to joystick inputs
        mecanumDev.jogMoveXYR(gamepad1.left_stick_x, -gamepad1.left_stick_y,-gamepad1.right_stick_x);
        // Slider moves manual according to joystick inputs when not in pick sample process
        if( pickUpSample == PickSample.pickIdle && cagePrePick == CagePrePick.cageIdle && cagePick == CagePick.cagePickIdle ) sliderDev.moveJog( -gamepad2.left_stick_y );
        // Run Pick Up Sample Sequence
        pickSpecimen();
        prePickSACage();
        pickSACage();
        if(gamepad2.dpad_left) gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
        /*
        if( turnerToggle.Toggle(gamepad2.triangle) )
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerFlipped);
        else
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);

         */


        pullUp.servoControl(gamepad1.circle);
        pullUp.motorControl(gamepad1.triangle, gamepad1.cross);
        // Time quanta
        dT = tm.milliseconds();
        tm.reset();
        // Call execution of all robot objects
        processAllSystems();

        // Telemetry
        //int red = hardware.colorSensor.red();
        //int blue = hardware.colorSensor.blue();
        //int green = hardware.colorSensor.green();

//        telemetry.addData("red", hardware.colorSensor.red());
//        telemetry.addData("green", hardware.colorSensor.green());
//        telemetry.addData("blue", hardware.colorSensor.blue());

        telemetry.addData("sliderSts:", sliderDev.Status);
        telemetry.addData("trgPos:", sliderDev.targetPos);
        telemetry.addData("actPos:", sliderDev.actPos);
        telemetry.addData("actSpe:", sliderDev.actSpeed);
        telemetry.addData("spSpe", sliderDev.sspSpeed );
        telemetry.addData("sPow:", sliderDev.sliderPower);
        //        telemetry.addData("spSpeed:", sliderDev.getSpSpeed());
//        telemetry.addData("actPos:", sliderDev.getActPosition());


        //telemetry.addData("Status:", cagePickUp );

        // telemetry.addData("transf.:", transferArm.isReady());
        // telemetry.addData("turner:", turnerArm.isReady());
        // telemetry.addData("handler:", handlerArm.isReady());


        telemetry.addData("P-Kp:", ConfigVar.Slider.POS_KP);
        telemetry.addData("S-Kp:", ConfigVar.Slider.SPEED_KP);
        telemetry.addData("S-Ki:", ConfigVar.Slider.SPEED_KI);
        telemetry.addData("S-Ki:", ConfigVar.Slider.SPEED_KD);
/*
        telemetry.addData("pole:", poleArm.isReady());
        telemetry.addData("sPos:", sliderDev.getActPosition());
        telemetry.addData("Pow:", sliderDev.sliderPower);
        telemetry.addData("actSpe:", sliderDev.actSpeed);
        //telemetry.addData("StsSample:", pickUpSample );
        telemetry.addData("dT:", sliderDev.dT);


        telemetry.addData("turner:", turnerArm.isReady());
        telemetry.addData("transf.:", transferArm.isReady());
        telemetry.addData("handler:", handlerArm.isReady());
        telemetry.addData("gripper:", gripperArm.isReady());
        telemetry.addData("pole:", poleArm.isReady());
*/

//        telemetry.addData("sInPos:", sliderDev.inPosition());
        telemetry.addData("Sts:", pickUpSample );
        telemetry.addData("PrePick", cagePrePick);
        telemetry.addData("Pick:", cagePick);

        telemetry.update();

    }
}
}