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

enum CagePrePick { cageIdle, cagePick1,cagePick2, cagePick3, cagePick4 };
CagePrePick cagePrePick = CagePrePick.cageIdle;
public void prePickSACage(){
    switch ( cagePrePick ) {
        case cageIdle:
            // Check all servo's are ready
            if (!gamepad2.dpad_left || sliderDev.notReady() || gripperArm.notReady()  || poleArm.notReady()) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            cagePrePick = CagePrePick.cagePick1;
            break;
        case cagePick1:
            if (sliderDev.notReady() || gripperArm.notReady()  || poleArm.notReady()) break;
            sliderDev.moveTo(ConfigVar.Slider.SA_HOME);
            // Set status for next step in sequence
            cagePrePick = CagePrePick.cagePick2;
            break;
        case cagePick2:
            if (sliderDev.notReady() || gripperArm.notReady()  || poleArm.notReady()) break;
            poleArm.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            cagePrePick = CagePrePick.cagePick3;
        case cagePick3:
            if (poleArm.notReady() || gripperArm.notReady() || sliderDev.notReady()) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
            cagePrePick = CagePrePick.cagePick4;
            break;
        case cagePick4:
            if ( !gamepad2.dpad_left && ( poleArm.notReady() || gripperArm.notReady())) break;
            poleArm.moveTo(ConfigVar.ArmCfg.poleHome);
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
            if (!gamepad2.dpad_right || gripperArm.notReady() || poleArm.notReady()) break;
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

enum PickUSample {pickIdle, pickStart, pickFlip, pickTransfer, pickPreGrab,pickGrab, pickSliderUp, pickPolePlace, pickToIdle }
PickUSample pickUpSample = PickUSample.pickIdle;

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
            if( !gamepad2.dpad_up || sliderDev.notReady() || handlerArm.notReady() || transferArm.notReady() || turnerArm.notReady() ) break;
            // Set status for next step in sequence
            pickUpSample = PickUSample.pickStart;
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerOpened); // Open gripper
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            poleArm.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            sliderDev.moveTo(ConfigVar.Slider.SP_PRE_PICK ); // Move slider to pre-pick position ( retracted up)
            break;
        case pickStart:
            if( poleArm.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            // Set status for next step in sequence
            pickUpSample = ( rgbSensor.getColorValue() != RGBSensor.ColorValue.blackValue )? PickUSample.pickFlip : PickUSample.pickTransfer;
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

    while ( opModeIsActive() )
    {
        // mecanumDev moves manual according to joystick inputs
        mecanumDev.jogMoveXYR(gamepad1.left_stick_x, gamepad1.left_stick_y,-gamepad1.right_stick_x);
        // Slider moves manual according to joystick inputs when not in pick sample process
        if( pickUpSample == PickUSample.pickIdle && cagePrePick == CagePrePick.cageIdle && cagePick == CagePick.cagePickIdle ) sliderDev.moveJog( -gamepad2.left_stick_y );
        // Run Pick Up Sample Sequence
        pickSpecimen();
        prePickSACage();
        pickSACage();

        // Time quanta
        dT = tm.milliseconds();
        tm.reset();
        // Call execution of all robot objects
        processAllSystems();

        // Telemetry
        int red = hardware.colorSensor.red();
        int blue = hardware.colorSensor.blue();
        int green = hardware.colorSensor.green();

        telemetry.addData("red", hardware.colorSensor.red());
        telemetry.addData("green", hardware.colorSensor.green());
        telemetry.addData("blue", hardware.colorSensor.blue());

        telemetry.addData("sliderState:", sliderDev.Status);
        telemetry.addData("trgPos:", sliderDev.targetPos);
        telemetry.addData("actSpeed:", sliderDev.getActSpeed());
        telemetry.addData("spSpeed:", sliderDev.getSpSpeed());
        telemetry.addData("actPos:", sliderDev.getActPosition());


        //telemetry.addData("Status:", cagePickUp );

        // telemetry.addData("transf.:", transferArm.isReady());
        // telemetry.addData("turner:", turnerArm.isReady());
        // telemetry.addData("handler:", handlerArm.isReady());

/*
        telemetry.addData("gripper:", gripperArm.isReady());
        telemetry.addData("pole:", poleArm.isReady());
        telemetry.addData("sPos:", sliderDev.getActPosition());
        telemetry.addData("Pow:", sliderDev.sliderPower);
        telemetry.addData("actSpe:", sliderDev.actSpeed);
        //telemetry.addData("StsSample:", pickUpSample );
        telemetry.addData("dT:", sliderDev.dT);
  */
        telemetry.addData("transf.:", transferArm.isReady());
        telemetry.addData("turner:", turnerArm.isReady());
        telemetry.addData("handler:", handlerArm.isReady());
        telemetry.addData("gripper:", gripperArm.isReady());
        telemetry.addData("pole:", poleArm.isReady());
        telemetry.addData("sPos:", sliderDev.getActPosition());
        telemetry.addData("sInPos:", sliderDev.inPosition());
        telemetry.addData("Sts:", pickUpSample );
        telemetry.addData("PrePick", cagePrePick);
        telemetry.addData("Pick:", cagePick);

        telemetry.update();

    }
}
}