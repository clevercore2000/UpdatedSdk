package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Preload Specimen Red")
public class SpPreloadRed extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Hardware hardware;
    //private ConfigVar configVar;
    private ArmDev gripperArm;
    private ArmDev handlerArm;
    private ArmDev poleArm1;
    private ArmDev poleArm2;
    private ArmDev transferArm;
    private ArmDev turnerArm;
    private ArmDev rotGripperArm;
    private SliderDev sliderDev;

    private int pathState;



    private final Pose startPose = new Pose(136, 65   , Math.toRadians(0));


    private final Pose scorePose = new Pose(110, 67.6, Math.toRadians(0));
    private final Pose scoreControlPose = new Pose(121,65.5, Math.toRadians(0) );
    private final Pose postScorePose = new Pose(130, 67, Math.toRadians(0));
    private final Pose fineScorePos = new Pose(110, 67, Math.toRadians(0));
    private final Pose parkPose = new Pose(137, 15, Math.toRadians(0));

    private final Pose pickUp1pos = new Pose(46, 35, Math.toRadians(270));
    private final Pose controlPickUp1pos = new Pose(19.212, 52.2);


    private Path scorePreload;
    private Path postScorePreload;
    private Path fineScore;
    private Path park;
    //private PathChain push1, push2, push3, park, prePark;
    public void buildPaths() {



        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(scoreControlPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        park = new Path(new BezierLine(new Point(postScorePose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(postScorePose.getHeading(), parkPose.getHeading());

        postScorePreload = new Path(new BezierLine(new Point(fineScorePos), new Point(postScorePose)));
        postScorePreload.setLinearHeadingInterpolation(fineScorePos.getHeading(), postScorePose.getHeading());

        fineScore = new Path(new BezierLine(new Point(scorePose), new Point(fineScorePos)));
        fineScore.setLinearHeadingInterpolation(scorePose.getHeading(), fineScorePos.getHeading());


        /*push1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(postScorePose), new Point(controlPickUp1pos), new Point(pickUp1pos)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();*/

    }
    enum PickSample {pickSp12,pickIdle, pickSP1 ,pickSP2, pickSP3, pickSP4, pickSP5, pickSP6, pickSP7, pickSP71, pickSP8, pickSP9, pickSP10 }
    PickSample pickUpSample = PickSample.pickIdle;
    boolean specimenStart = false;

    public void pickSpecimen()
    {
        switch( pickUpSample )
        {
            case pickIdle:
                if( !specimenStart ) break;
                if( sliderDev.notReady() ) break;


                gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
                handlerArm.moveTo(ConfigVar.ArmCfg.handlerOpened);
                rotGripperArm.moveTo( 150 );
                sliderDev.moveTo(ConfigVar.Slider.SP_PRE_PICK +500);

                //tm.reset();
                pickUpSample = PickSample.pickSp12;
                break;
            case pickSp12:
                if( rotGripperArm.notReady() && poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
                    transferArm.moveTo(ConfigVar.ArmCfg.transferSpCoop);
            case pickSP1:
                if( rotGripperArm.notReady() && poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;

                poleArm1.moveTo(ConfigVar.ArmCfg.poleSpPrePick);
                poleArm2.moveTo(ConfigVar.ArmCfg.poleSpPrePick);
                pickUpSample = PickSample.pickSP3;
                break;
            case pickSP2:
                if( poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;

                if(hardware.sliderMotor2.getCurrentPosition() < ConfigVar.Slider.SP_PICK ) sliderDev.moveTo(ConfigVar.Slider.SP_PRE_PICK-100); // Move slider to pre-pick position ( retracted up)

                pickUpSample = PickSample.pickSP3;
                break;
            case pickSP3:
                if( poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
                handlerArm.moveTo(ConfigVar.ArmCfg.handlerOpened);
                pickUpSample = PickSample.pickSP5;
                break;
            case pickSP4: // Flip Turner
                if( poleArm1.notReady() || poleArm2.notReady() || transferArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;

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
                sliderDev.moveTo(ConfigVar.Slider.SP_PICK-50 );
                turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
                // Set status for next step in sequence
                pickUpSample = PickSample.pickSP7;
                break;
            case pickSP7:
                if( sliderDev.notReady() ) break; // Wait slider to complete it's move
                handlerArm.moveTo(ConfigVar.ArmCfg.handlerClosed);
                // Set status for next step in sequence
                pickUpSample = PickSample.pickSP71;
                break;
            case pickSP71:
                if(handlerArm.notReady())break;
                transferArm.moveTo(ConfigVar.ArmCfg.transferSpPreCoop);
                pickUpSample = PickSample.pickSP8;
                break;
            case pickSP8:
                if( sliderDev.notReady() || handlerArm.notReady() || transferArm.notReady()) break; // Wait slider to complete it's move
                sliderDev.moveTo(3050 );
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
                transferArm.moveTo(ConfigVar.ArmCfg.transferSpPreCoop);
                pickUpSample = PickSample.pickIdle;
                specimenStart = false;
                break;

        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                specimenStart = true;
                pathState = 1;
                break;
            case 1:
                if( specimenStart ) break;
                follower.followPath(scorePreload);
                pathState = 2;
                break;
            case 12:
                if(follower.isBusy()) break;

                follower.followPath(fineScore);
                pathState = 2;
                break;
            case 2:
                if( follower.isBusy() ) break;

                sliderDev.moveTo( 3400, 1000 );
               // follower.followPath(fineScore);

                pathState = 3;
                break;
            case 3:
                if( sliderDev.notReady() || follower.isBusy()) break;

                follower.followPath(postScorePreload);
                pathState = 4;
                break;
            case 4:
                if(follower.isBusy()) break;

                poleArm1.moveTo(ConfigVar.ArmCfg.poleSpPrePick);
                poleArm2.moveTo(ConfigVar.ArmCfg.poleSpPrePick);
                sliderDev.moveTo(ConfigVar.Slider.SA_HOME);
                pathState = 5;
                break;
            case 5:
                if(poleArm1.notReady() || poleArm2.notReady()||sliderDev.notReady()) break;
                follower.followPath(park);
                pathState = 98;
                break;
            case 98:
                if(follower.isBusy()) break;
                poleArm1.moveTo(ConfigVar.ArmCfg.poleIdle);
                poleArm2.moveTo(ConfigVar.ArmCfg.poleIdle);
                pathState = 99;
                break;
            case 99:
                if(poleArm1.notReady()||poleArm2.notReady())break;
                sliderDev.moveTo(-200);
                break;
        }
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        poseUpdater.update();
        dashboardPoseTracker.update();
        pickSpecimen();
        sliderDev.execute();
        handlerArm.execute();
        transferArm.execute();
        gripperArm.execute();
        poleArm1.execute();
        poleArm2.execute();
        transferArm.execute();
        rotGripperArm.execute();



        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("sample state", pickUpSample);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("specStart", specimenStart);
        telemetry.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {




        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        hardware = new Hardware(hardwareMap);
        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        sliderDev = new SliderDev(hardware);
        gripperArm = new ArmDev( hardware.gripperServo, ConfigVar.ArmCfg.GRIPPER_SPEED);
        handlerArm = new ArmDev(hardware.handlerServo, ConfigVar.ArmCfg.HANDLER_SPEED);
        poleArm1 = new ArmDev(hardware.poleServo1, ConfigVar.ArmCfg.POLE_SPEED );
        transferArm = new ArmDev(hardware.transferServo, ConfigVar.ArmCfg.TRANSFER_SPEED);
        turnerArm = new ArmDev(hardware.turnerServo, ConfigVar.ArmCfg.TURNER_SPEED);
        poleArm2 = new ArmDev(hardware.poleServo2, ConfigVar.ArmCfg.POLE_SPEED);
        rotGripperArm = new ArmDev(hardware.rotateGripper,ConfigVar.ArmCfg.ROT_GRIPPER_SPEED);

        //setHomePositions();


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        pathState = 0;
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        setHomePositions();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public void setHomePositions()
    {
        // .. Set Home/Idle positions of all systems ...

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
       // rotGripperArm.moveTo( 150 );

        sliderDev.moveTo(ConfigVar.Slider.MIN_HEIGHT);

    }

}

