package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
private ConfigVar configVar;
private SliderDev sliderDev;
private MecanumDev mecanumDev;
// Elaps timer
private ElapsedTime tm;
private double dt;

RCodeParser Parser;
public void initialize()
{
    // ... initialization logic ...
    try {
        hardware = new Hardware(hardwareMap);
        configVar = new ConfigVar();

        sliderDev = new SliderDev(hardware);
        mecanumDev = new MecanumDev(hardware);
        tm = new ElapsedTime();

        gripperArm = new ArmDev( hardware.gripperServo, 240);

        handlerArm = new ArmDev(hardware.handlerServo, 10);
        poleArm = new ArmDev(hardware.poleServo, 200 );
        transferArm = new ArmDev(hardware.transferServo, 10);
        turnerArm = new ArmDev(hardware.turnerServo, 10);

        tm.startTime();
    } catch (Exception e) {
        throw new RuntimeException(e);
    }
}

public void setHomePositions()
{
    // .. Set Home/Idle positions of all systems ...
    mecanumDev.Initialize();

    sliderDev.Initialize();
    // sliderDev.Status = SliderDev.SliderStatus.SilderMoveJog;

    gripperArm.setRange(ConfigVar.ArmCfg.GRIPPER_MIN, ConfigVar.ArmCfg.GRIPPER_MAX);
    gripperArm.moveTo( ConfigVar.ArmCfg.gripperOpened);
/*
    handlerArm.setRange(ConfigVar.ArmCfg.HANDLER_MIN, ConfigVar.ArmCfg.HANDLER_MAX);
    handlerArm.moveTo( ConfigVar.ArmCfg.handlerClosed);

    transferArm.setRange(ConfigVar.ArmCfg.TRANSFER_MIN, ConfigVar.ArmCfg.TRANSFER_MAX);
    transferArm.moveTo( ConfigVar.ArmCfg.transferSpPreCoop);
*/
    poleArm.setRange(ConfigVar.ArmCfg.POLE_MIN,ConfigVar.ArmCfg.POLE_MAX);
    poleArm.moveTo( ConfigVar.ArmCfg.poleHome);
/*
    turnerArm.setRange(ConfigVar.ArmCfg.TURNER_MIN,ConfigVar.ArmCfg.TURNER_MAX);
    turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
*/
}

void processAllSystems()
{
    mecanumDev.execute();
   // sliderDev.execute();
   // gripperArm.execute();
    //handlerArm.execute();
   // poleArm.execute();
   // transferArm.execute();
   // turnerArm.execute();
}

enum ColorValue {redValue, yellowValue, blueValue, blackValue, otherValue};
ColorValue colorValue;
enum PickUSample {pickIdle, pickStart, pickFlip, pickTransfer, pickPreGrab,pickGrab, pickToIdle }
PickUSample pickUpSample = PickUSample.pickIdle;

ColorValue getColorValue()
{
    int red = hardware.colorSensor.red();
    int blue = hardware.colorSensor.blue();
    int green = hardware.colorSensor.green();
    if( red > 200 && green > 200 && blue < 25 ) return ColorValue.yellowValue;
    if( red > 200 && green < 25 && blue < 25 ) return ColorValue.redValue;
    if( red < 25 && green < 25 && blue < 200 ) return ColorValue.blueValue;
    if( red < 25 && green < 25 && blue < 25 ) return ColorValue.blackValue;
    return ColorValue.otherValue;
}
/*
This method implements the sequence to use SliderArm, handlerArm, transferArm, turnerArm and colorSensor
to detect flip if needed, transfer and grab the sample
*/
public void pickSample()
{
    switch( pickUpSample )
    {
        case pickIdle:
            // Wait to push the start button
            //if( !gamepad2.dpad_down ) break;// Driver request to pick up sample
            // Check all servo's are ready
            if( sliderDev.notReady() || handlerArm.notReady() || transferArm.notReady() || turnerArm.notReady() ) break;
            // Set status for next step in sequence
            pickUpSample = PickUSample.pickStart;
            // Open gripper
            handlerArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
            // Move slider to pre-pick position ( retracted up)
            sliderDev.moveTo(ConfigVar.Slider.SP_PRE_PICK );
        case pickStart:
            if( handlerArm.notReady() || sliderDev.notReady() ) break;
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
            transferArm.moveTo(ConfigVar.ArmCfg.transferSpPreCoop);
            // Set status for next step in sequence
            pickUpSample = ( hardware.colorSensor.red() == 200  )? PickUSample.pickFlip : PickUSample.pickTransfer;
            break;
        case pickFlip:
            if( transferArm.notReady() ) break;
            // Flip the sample
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerFlipped);
            // Set status for next step in sequence
            pickUpSample = PickUSample.pickTransfer;
            break;
        case pickTransfer:
            if( turnerArm.notReady() ) break;
            transferArm.moveTo(ConfigVar.ArmCfg.transferSpCoop);
            // Set status for next step in sequence
            pickUpSample = PickUSample.pickPreGrab;
            break;
        case pickPreGrab:
            if( transferArm.notReady() ) break;
            sliderDev.moveTo(ConfigVar.Slider.SP_PICK );
            // We can also move the Turner back in Idle position
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
            // Set status for next step in sequence
            pickUpSample = PickUSample.pickGrab;
            break;
        case pickGrab:
            if( sliderDev.notReady() ) break; // Wait slider to complete it's move
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerClosed);
            //transferArm.moveTo(ConfigVar.ArmCfg.transferSpPreCoop);
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
            // Set status for next step in sequence
            pickUpSample = PickUSample.pickIdle;
            break;
        case pickToIdle:
            if( handlerArm.notReady() ) break; // Wait for handler to grab the sample
            // Return the transferArm to pre-coop position
            transferArm.moveTo(ConfigVar.ArmCfg.transferSpPreCoop);
            // All pickSample operation completed - go back to Idle
            pickUpSample = PickUSample.pickIdle;
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

    while (opModeIsActive())
    {
        // mecanumDev moves manual according to joystick inputs
        mecanumDev.jogMoveXYR(-gamepad1.left_stick_x, gamepad1.left_stick_y,-gamepad1.right_stick_x);
        // Slider moves manual according to joystick inputs when not in pick sample process
        if( pickUpSample == PickUSample.pickIdle) sliderDev.moveJog( -gamepad2.left_stick_y );
// Pick sample system is now programmed in a sequence
// Use joystick only for tests/debugging the systems
//        if( gamepad2.dpad_up) gripperArm.moveTo(( gripperToggle.Toggle(gamepad2.dpad_up) )? ConfigVar.ArmCfg.gripperClosed : ConfigVar.ArmCfg.gripperOpened );
//        if( gamepad2.dpad_left ) handlerArm.moveTo((transferToggle.Toggle(gamepad2.dpad_left))? ConfigVar.ArmCfg.handlerClosed : ConfigVar.ArmCfg.handlerOpened);
//        if( gamepad2.dpad_down) poleArm.moveTo((poleToggle.Toggle(gamepad2.dpad_down))? ConfigVar.ArmCfg.poleSaPick : ConfigVar.ArmCfg.poleSaPrePick);
//        if( gamepad2.circle ) transferArm.moveTo((transferToggle.Toggle(gamepad2.circle))? ConfigVar.ArmCfg.transferSpCoop : ConfigVar.ArmCfg.transferSpPreCoop);
//        if( gamepad2.dpad_right ) turnerArm.moveTo((transferToggle.Toggle(gamepad2.dpad_right))? ConfigVar.ArmCfg.turnerIdle : ConfigVar.ArmCfg.turnerFlipped);

        // Run Pick Up Sample Sequence
        if( gamepad2.dpad_down ) pickSample();
        // Time quanta
        dt = tm.milliseconds();
        tm.reset();
        // Call execution of all robot objects
        sliderDev.execute();
        mecanumDev.execute();
        gripperArm.execute();
        poleArm.execute();
        turnerArm.execute();
        handlerArm.execute();
        // Telemetry
        int red = hardware.colorSensor.red();
        int blue = hardware.colorSensor.blue();
        int green = hardware.colorSensor.green();

        telemetry.addData("red", hardware.colorSensor.red());
        telemetry.addData("green", hardware.colorSensor.green());
        telemetry.addData("blue", hardware.colorSensor.blue());
        telemetry.addData("s_tgS:", sliderDev.getTargetSpeed());
        telemetry.update();

    }
}
}