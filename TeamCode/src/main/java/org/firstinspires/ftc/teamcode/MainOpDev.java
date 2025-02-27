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
private ArmDev poleArm1;
private ArmDev poleArm2;
private ArmDev transferArm;
private ArmDev turnerArm;
private ArmDev rotGripperArm;
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
enum  InitStatus {notStarted, initDone};
InitStatus initStatus = InitStatus.notStarted;
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
        poleArm1 = new ArmDev(hardware.poleServo1, ConfigVar.ArmCfg.POLE_SPEED );
        transferArm = new ArmDev(hardware.transferServo, ConfigVar.ArmCfg.TRANSFER_SPEED);
        turnerArm = new ArmDev(hardware.turnerServo, ConfigVar.ArmCfg.TURNER_SPEED);
        poleArm2 = new ArmDev(hardware.poleServo2, ConfigVar.ArmCfg.POLE_SPEED);
        rotGripperArm = new ArmDev(hardware.rotateGripper,ConfigVar.ArmCfg.ROT_GRIPPER_SPEED);
        gripperToggle = new ToggleButton();
        handleToggle = new ToggleButton();
        transferToggle = new ToggleButton();
        poleToggle = new ToggleButton();
        turnerToggle = new ToggleButton();
        moveToSlider = new ToggleButton();

        rgbSensor = new RGBSensor( hardware );
        tm.startTime();
        initStatus = InitStatus.initDone;
    } catch (Exception e) {
        throw new RuntimeException(e);
    }
}

public void setHomePositions()
{
    // .. Set Home/Idle positions of all systems ...
    mecanumDev.Initialize();

    gripperArm.setRange(ConfigVar.ArmCfg.GRIPPER_MIN, ConfigVar.ArmCfg.GRIPPER_MAX);
    gripperArm.moveTo( ConfigVar.ArmCfg.gripperOpened);

    handlerArm.setRange(ConfigVar.ArmCfg.HANDLER_MIN, ConfigVar.ArmCfg.HANDLER_MAX);
    handlerArm.moveTo( ConfigVar.ArmCfg.handlerClosed);

    transferArm.setRange(ConfigVar.ArmCfg.TRANSFER_MIN, ConfigVar.ArmCfg.TRANSFER_MAX);
    transferArm.moveTo( ConfigVar.ArmCfg.transferSpPreCoop);

    poleArm1.setRange(ConfigVar.ArmCfg.POLE_MIN1,ConfigVar.ArmCfg.POLE_MAX1);
    poleArm1.moveTo( ConfigVar.ArmCfg.poleIdle);

    poleArm2.setRange(ConfigVar.ArmCfg.POLE_MIN2,ConfigVar.ArmCfg.POLE_MAX2);
    poleArm2.moveTo( ConfigVar.ArmCfg.poleIdle);

    turnerArm.setRange(ConfigVar.ArmCfg.TURNER_MIN,ConfigVar.ArmCfg.TURNER_MAX);
    turnerArm.moveTo( ConfigVar.ArmCfg.turnerIdle);

    rotGripperArm.setRange( ConfigVar.ArmCfg.ROT_GRIPPER_MIN,ConfigVar.ArmCfg.ROT_GRIPPER_MAX);
    rotGripperArm.moveTo( 150 );

    sliderDev.moveTo(ConfigVar.Slider.MIN_HEIGHT);

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
    rotGripperArm.execute();
}

enum CagePrePick { cageIdle, cagePick1,cagePick2, cagePick3, cagePick4, cagePick5, cagePick6, cagePick7 };
CagePrePick cagePrePick = CagePrePick.cageIdle;
public void prePickSACage(){
    switch ( cagePrePick ) {
        case cageIdle:
            if (!gamepad2.square || sliderDev.notReady() || gripperArm.notReady()  || poleArm1.notReady() || poleArm2.notReady()) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerClosed);
            rotGripperArm.moveTo( 150 );
            if(hardware.sliderMotor1.getCurrentPosition() < ConfigVar.Slider.SA_HOME + 300) sliderDev.moveTo(ConfigVar.Slider.SP_PRE_PICK);
            cagePrePick = CagePrePick.cagePick1;
            break;
        case cagePick1:
            if( sliderDev.notReady() || gripperArm.notReady() || rotGripperArm.notReady() || handlerArm.notReady() ) break;
            //if( tm.seconds() < 1.0) break;
            cagePrePick = CagePrePick.cagePick2;
        case cagePick2:
            if (sliderDev.notReady() || gripperArm.notReady() || handlerArm.notReady()  || poleArm1.notReady() || poleArm2.notReady()) break;
            poleArm1.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            poleArm2.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            cagePrePick = CagePrePick.cagePick3;
            break;
        case cagePick3:
            if (sliderDev.notReady() || gripperArm.notReady()  || poleArm1.notReady() || poleArm2.notReady()) break;
            sliderDev.moveTo(ConfigVar.Slider.SA_PRE_PICK);
            cagePrePick = CagePrePick.cagePick4;
        case cagePick4:
            if (poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() || sliderDev.notReady()) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
            cagePrePick = CagePrePick.cagePick5;
            break;
        case cagePick5:
            if ( !gamepad2.square || poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() ) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            //poleArm.moveTo(ConfigVar.ArmCfg.poleHome);
            cagePrePick = CagePrePick.cagePick6;
            break;
        case cagePick6:
            if (poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() || sliderDev.notReady()) break;
            sliderDev.moveTo(ConfigVar.Slider.TOP_BASCKET_POS);
            poleArm1.moveTo(ConfigVar.ArmCfg.poleSaPlace2);
            poleArm2.moveTo(ConfigVar.ArmCfg.poleSaPlace2);
            cagePrePick = CagePrePick.cageIdle;
            break;
    }
}

enum CagePick { cagePickIdle, cageGripper, cagePick,cagePrePick, cageOpenGripper}
CagePick cagePick = CagePick.cagePickIdle;
public void pickSACage()
{
    switch ( cagePick ) {
        case cagePickIdle:
            // Check all servo's are ready
            if (!(gamepad2.circle && cagePrePick == CagePrePick.cagePick5)|| gripperArm.notReady() || poleArm1.notReady() || poleArm2.notReady()) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
            cagePick = CagePick.cageGripper;
            break;
        case cageGripper:
            if (gripperArm.notReady() || poleArm1.notReady() || poleArm2.notReady()) break;
            poleArm1.moveTo(ConfigVar.ArmCfg.poleSaPick);
            poleArm2.moveTo(ConfigVar.ArmCfg.poleSaPick);
            cagePick = CagePick.cagePick;
            break;
        case cagePick:
            // Check all servo's are ready
            if (gripperArm.notReady() || poleArm1.notReady() || poleArm2.notReady()) break;
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            // Set status for next step in sequence
            cagePick = CagePick.cagePrePick;
            break;
        case cagePrePick:
            if (gripperArm.notReady() || poleArm1.notReady() || poleArm2.notReady()) break;
            poleArm1.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            poleArm2.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
            cagePick = CagePick.cagePickIdle;
            break;
    }
}

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
//            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerOpened);
            rotGripperArm.moveTo( 150 );
            pickUpSample = PickSample.pickSP1;
            break;
        case pickSP1:
            if( rotGripperArm.notReady() && poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            poleArm1.moveTo(ConfigVar.ArmCfg.poleSpPrePick);
            poleArm2.moveTo(ConfigVar.ArmCfg.poleSpPrePick);
            pickUpSample = PickSample.pickSP2;
            break;
        case pickSP2:
            if( poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            if(hardware.sliderMotor2.getCurrentPosition() < ConfigVar.Slider.SP_PICK ) sliderDev.moveTo(ConfigVar.Slider.SP_PICK + 200); // Move slider to pre-pick position ( retracted up)
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerOpened);
            pickUpSample = PickSample.pickSP5;
            break;
            case pickSP3:
            if( poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            //pickUpSample = ( rgbSensor.getColorValue() != RGBSensor.ColorValue.blackValue )? PickSample.pickSP4 : PickSample.pickSP5;
            handlerArm.moveTo(ConfigVar.ArmCfg.handlerOpened);
            pickUpSample = PickSample.pickSP5;
            break;
        case pickSP4: // Flip Turner
            if( poleArm1.notReady() || poleArm2.notReady() || transferArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
            turnerArm.moveTo(ConfigVar.ArmCfg.turnerFlipped); // Flip the sample
            // Set status for next step in sequence
            pickUpSample = PickSample.pickSP5;
            break;
        case pickSP5:
            if( poleArm1.notReady() || poleArm2.notReady() || transferArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
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
            if( sliderDev.notReady() || handlerArm.notReady() || transferArm.notReady()) break; // Wait slider to complete it's move
            sliderDev.moveTo(ConfigVar.Slider.SP_PLACE );
            gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
            rotGripperArm.moveTo( 150 );
            pickUpSample = PickSample.pickSP9;
            break;
        case pickSP9:
            if(  sliderDev.notReady() || rotGripperArm.notReady()) break; // Wait slider to complete it's move
            poleArm1.moveTo(ConfigVar.ArmCfg.poleSpPlace);
            poleArm2.moveTo(ConfigVar.ArmCfg.poleSpPlace);
            pickUpSample = PickSample.pickSP10;
        case pickSP10:
            if( poleArm1.notReady() || poleArm2.notReady() ) break; // Wait for handler to grab the sample
            // Return the transferArm to pre-coop position
            transferArm.moveTo(ConfigVar.ArmCfg.transferSpPreCoop);
            // All pickSample operation completed - go back to Idle
            pickUpSample = PickSample.pickIdle;
            break;
    }
}

void killSwitch()
{
    if( gamepad2.right_bumper) // Engage kill switch
    {
        pickUpSample = PickSample.pickIdle;
        cagePrePick = CagePrePick.cageIdle;
        cagePick = CagePick.cagePickIdle;
        sliderDev.stop();
    }
}

public void orientGripper()
{
    double jy = -gamepad2.right_stick_y;
    double jx = gamepad2.right_stick_x;
    double theta = Math.toDegrees( Math.atan2(jy,jx) );
    double radiusPos = Math.sqrt( (jx * jx) + (jy * jy) );

    if( radiusPos > 0.8 && theta > 0  ) rotGripperArm.moveTo( (240-theta) );
    telemetry.addData("Theta:",theta);
    telemetry.addData("R:",radiusPos);
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
        //if( initStatus == InitStatus.homesDone) initStatus = InitStatus.opStarted;
        // Kill switch stops all sequences and stops the slider
        killSwitch();
        // mecanumDev moves manual according to joystick inputs
        mecanumDev.jogMoveXYR(-gamepad1.left_stick_x, -gamepad1.left_stick_y,-gamepad1.right_stick_x);
        // Slider moves manual according to joystick inputs when not in pick sample process
        if( pickUpSample == PickSample.pickIdle && cagePrePick == CagePrePick.cageIdle && cagePick == CagePick.cagePickIdle ) sliderDev.moveJog( -gamepad2.left_stick_y );
        // Run Pick Up Sample Sequence
        pickSpecimen();
        prePickSACage();
        pickSACage();
        if(gamepad2.dpad_left) gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
        if(gamepad2.dpad_up) {poleArm1.moveTo(ConfigVar.ArmCfg.poleSpPrePick);
            poleArm2.moveTo(ConfigVar.ArmCfg.poleSpPrePick);}
        //
        orientGripper();

        pullUp.servoControl(gamepad1.circle);
        pullUp.motorControl(gamepad1.triangle, gamepad1.cross);
        // Time quanta
        dT = tm.milliseconds();
        tm.reset();
        // Call execution of all robot objects
        processAllSystems();

        // Telemetry
       // telemetry.addData("Sts:", pickUpSample );
        //telemetry.addData("Init:",initStatus);
        //telemetry.addData("PrePick", cagePrePick);
        //telemetry.addData("Pick:", cagePick);
/*
        telemetry.addData("whPos", mecanumDev.odo.actWheelsPos[0]);
        telemetry.addData("whSpe", mecanumDev.odo.actWheelsSpeed[0]);
        telemetry.addData("Pos:" , mecanumDev.odo.actPos[0]);
        telemetry.addData("Spe" , mecanumDev.odo.actSpeed[0]);*/
        telemetry.addData("sPos:", sliderDev.actPos);

//        telemetry.addData("gripper:", hardware.gripperServo.getPosition());
     telemetry.addData("sliderSts:", sliderDev.Status);
     telemetry.addData("trgPos:", sliderDev.targetPos);
     telemetry.addData("actSpe:", sliderDev.actSpeed);
//        telemetry.addData("spSpe", sliderDev.sspSpeed );
    telemetry.addData("sPow:", sliderDev.sliderPower);
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


/*
        telemetry.addData("turner:", turnerArm.isReady());
        telemetry.addData("transf.:", transferArm.isReady());
        telemetry.addData("handler:", handlerArm.isReady());
        telemetry.addData("gripper:", gripperArm.isReady());
        telemetry.addData("pole:", poleArm.isReady());
*/

//        telemetry.addData("sInPos:", sliderDev.inPosition());


        telemetry.update();

    }
}
}