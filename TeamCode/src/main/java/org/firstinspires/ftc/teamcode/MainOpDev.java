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

ToggleButton gripperToggle;
ToggleButton handleToggle;
ToggleButton transferToggle;
ToggleButton turnerToggle;
ToggleButton poleToggle;

public void initialize()
{
    // ... initialization logic ...
    try {
        hardware = new Hardware(hardwareMap);
        configVar = new ConfigVar();

        sliderDev = new SliderDev(hardware);
        mecanumDev = new MecanumDev(hardware);
        tm = new ElapsedTime();

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
//    gripperArm.moveTo( ConfigVar.ArmCfg.gripperOpened);

    handlerArm.setRange(ConfigVar.ArmCfg.HANDLER_MIN, ConfigVar.ArmCfg.HANDLER_MAX);
//    handlerArm.moveTo( ConfigVar.ArmCfg.handlerClosed);

    transferArm.setRange(ConfigVar.ArmCfg.TRANSFER_MIN, ConfigVar.ArmCfg.TRANSFER_MAX);
    transferArm.moveTo( ConfigVar.ArmCfg.transferSpPreCoop);

    poleArm.setRange(ConfigVar.ArmCfg.POLE_MIN,ConfigVar.ArmCfg.POLE_MAX);
//    poleArm.moveTo( ConfigVar.ArmCfg.poleHome);

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
    };

enum CagePickUp { cageIdle,cagePrePick,  cageGripperOpen, cagePick, cagePoleHome };
CagePickUp cagePickUp = CagePickUp.cageIdle;

public void pickCage() {
    switch ( cagePickUp ) {
        case cageIdle:
            // Check all servo's are ready
            if (!gamepad2.dpad_left && (sliderDev.notReady() || gripperArm.notReady()  || poleArm.notReady())) break;
            sliderDev.moveTo(ConfigVar.Slider.SA_HOME);
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            cagePickUp = CagePickUp.cagePrePick;
            break;
        case cagePrePick:
            if (gripperArm.notReady() || sliderDev.notReady()) break;
            poleArm.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            cagePickUp = CagePickUp.cageGripperOpen;
        case cageGripperOpen:
            if (poleArm.notReady() ) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
            cagePickUp = CagePickUp.cagePick;
            break;
        case cagePick:
            /*Driver asks to Pick*/
            if ( !gamepad2.dpad_right ||  poleArm.notReady()) break;
            cagePickUp = CagePickUp.cagePoleHome;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            // Set status for next step in sequence
            break;
        case cagePoleHome:
            if (gripperArm.notReady()) break;
            cagePickUp = CagePickUp.cageIdle;
            poleArm.moveTo(ConfigVar.ArmCfg.poleHome);
            break;
    }
}

enum ColorValue {redValue, yellowValue, blueValue, blackValue, otherValue};
ColorValue colorValue;
enum PickUSample {pickIdle, pickStart, pickFlip, pickTransfer, pickPreGrab,pickGrab, pickSliderUp, pickPolePlace, pickToIdle }
PickUSample pickUpSample = PickUSample.pickIdle;

/*
This method implements the sequence to use SliderArm, handlerArm, transferArm, turnerArm and colorSensor
to detect flip if needed, transfer and grab the sample
*/
public void pickSample()
{
    switch( pickUpSample )
    {
        case pickIdle:
            // Check all servo's are ready
            if( !gamepad2.dpad_up || sliderDev.notReady() || handlerArm.notReady() || transferArm.notReady() || turnerArm.notReady() ) break;
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerOpened); // Open gripper
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            poleArm.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            sliderDev.moveTo(ConfigVar.Slider.SP_PRE_PICK ); // Move slider to pre-pick position ( retracted up)
            // Set status for next step in sequence
            pickUpSample = PickUSample.pickStart;
            break;
        case pickStart:
            if( poleArm.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            // Set status for next step in sequence
            pickUpSample = ( hardware.colorSensor.red() == 200 || hardware.colorSensor.blue() == 200 )? PickUSample.pickFlip : PickUSample.pickTransfer;
            break;
        case pickFlip:
            if( transferArm.notReady() ) break;
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerFlipped); // Flip the sample
            // Set status for next step in sequence
            pickUpSample = PickUSample.pickTransfer;
            break;
        case pickTransfer:
            if( turnerArm.notReady() || transferArm.notReady() ) break;
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
            if( sliderDev.notReady() || turnerArm.notReady() ) break; // Wait slider to complete it's move
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerClosed);
            // Set status for next step in sequence
            pickUpSample = PickUSample.pickSliderUp;
            break;
        case pickSliderUp:
            if( sliderDev.notReady() || handlerArm.notReady()) break; // Wait slider to complete it's move
            sliderDev.moveTo(ConfigVar.Slider.SP_PLACE );
            pickUpSample = PickUSample.pickPolePlace;
            break;
        case pickPolePlace:
            if( !gamepad2.dpad_up && sliderDev.notReady() ) break; // Wait slider to complete it's move
            poleArm.moveTo(ConfigVar.ArmCfg.poleSpPlace );
            pickUpSample = PickUSample.pickToIdle;
        case pickToIdle:
            if( poleArm.notReady() ) break; // Wait for handler to grab the sample
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
        mecanumDev.jogMoveXYR(gamepad1.left_stick_x, gamepad1.left_stick_y,-gamepad1.right_stick_x);
        // Slider moves manual according to joystick inputs when not in pick sample process
        if( pickUpSample == PickUSample.pickIdle && cagePickUp == CagePickUp.cageIdle ) sliderDev.moveJog( -gamepad2.left_stick_y );
        // Run Pick Up Sample Sequence
        pickSample();
        pickCage();
        // Use joystick only for tests/debugging the systems
/*
        if(  gripperToggle.Toggle(gamepad2.dpad_up) )
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed );
        else
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened );

        if(handleToggle.Toggle(gamepad2.dpad_left))
            handlerArm.moveTo( ConfigVar.ArmCfg.handlerClosed );
        else
            handlerArm.moveTo( ConfigVar.ArmCfg.handlerOpened );

        if( poleToggle.Toggle(gamepad2.dpad_down ))
            poleArm.moveTo( ConfigVar.ArmCfg.poleSaPick);
        else
            poleArm.moveTo(ConfigVar.ArmCfg.poleSaPrePick);

        if( transferToggle.Toggle(gamepad2.circle) )
            transferArm.moveTo(ConfigVar.ArmCfg.transferSpCoop);
        else
            transferArm.moveTo( ConfigVar.ArmCfg.transferSpPreCoop );

        if( turnerToggle.Toggle(gamepad2.triangle) )
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
        else
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerFlipped);
*/
        // Time quanta
        dt = tm.milliseconds();
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
        /*
        telemetry.addData("Wact:", hardware.sliderMotor1.getPower());//sliderDev.getActSpeed());
        telemetry.addData("Wsp:", sliderDev.getSpSpeed());
        telemetry.addData("Pact:", sliderDev.getActPosition());
        */

        telemetry.addData("SampleSts:", pickUpSample );
        telemetry.addData("CageSts:", cagePickUp );

        // telemetry.addData("transf.:", transferArm.isReady());
        // telemetry.addData("turner:", turnerArm.isReady());
        // telemetry.addData("handler:", handlerArm.isReady());
        /*
        telemetry.addData("gripper:", gripperArm.isReady());
        telemetry.addData("pole:", poleArm.isReady());
        telemetry.addData("sPos:", sliderDev.getActPosition());
        telemetry.addData("Pow:", sliderDev.slider1Power);
        telemetry.addData("actSpe:", sliderDev.actSpeed);
        //telemetry.addData("StsSample:", pickUpSample );
        telemetry.addData("dT:", sliderDev.dT);
        */
        telemetry.addData("transf.:", transferArm.isReady());
        telemetry.addData("turner:", turnerArm.isReady());
        telemetry.addData("handler:", handlerArm.isReady());
        telemetry.addData("gripper:", gripperArm.isReady());
        telemetry.addData("pole:", poleArm.isReady());
        telemetry.addData("slider.:", sliderDev.isReady());
        telemetry.addData("sPos:", sliderDev.getActPosition());
        telemetry.addData("sInPos:", sliderDev.inPosition());

        telemetry.update();

    }
}
}