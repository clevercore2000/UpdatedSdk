package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

@Autonomous(name = "Example Auto Blue", group = "Examples")
public class TestSpecimen extends OpMode {

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

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 78, Math.toRadians(180));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(46, 78, Math.toRadians(180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }
    enum PickSample {cancel,pickIdle, pickSP1 ,pickSP2, pickSP3, pickSP4, pickSP5, pickSP6, pickSP7, pickSP71, pickSP8, pickSP9, pickSP10 }
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
                handlerArm.moveTo(ConfigVar.ArmCfg.handlerClosed);
                rotGripperArm.moveTo( 150 );
                sliderDev.moveTo(ConfigVar.Slider.SP_PICK + 200);

                //tm.reset();
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

                pickUpSample = PickSample.pickSP3;
                break;
            case pickSP3:
                if( poleArm1.notReady() || poleArm2.notReady() || gripperArm.notReady() || handlerArm.notReady() || sliderDev.notReady() ) break;
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
                transferArm.moveTo(ConfigVar.ArmCfg.transferSpPreCoop);
                pickUpSample = PickSample.pickIdle;
                specimenStart = false;
                break;

        }
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                specimenStart = true;
                pathState = 50;
                break;
            case 50:
                if( specimenStart ) break;
                follower.followPath(scorePreload);
                pathState = 51;
                break;
            case 51:
                if( follower.isBusy() ) break;
                sliderDev.moveTo( 2500/*de verificat valoarea 2500 */);
                pathState = 52;
                break;
            case 52:
                if( sliderDev.notReady() ) break;
                //pathState = 53;
                break;

                      /* case 1:




                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if(!follower.isBusy()) {
                    //Score Preload

                    //Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                //This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position
                if(!follower.isBusy()) {
                     //Grab Sample

                    //Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if(!follower.isBusy()) {
                    //Score Sample

                    // Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {


                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:

                if(!follower.isBusy()) {

                    //Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:

                if(!follower.isBusy()) {

                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {

                    follower.followPath(park,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {


                    setPathState(-1);
                }
                break; */
        }
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        pickSpecimen();
        sliderDev.execute();
        handlerArm.execute();
        transferArm.execute();
        gripperArm.execute();
        poleArm1.execute();
        poleArm2.execute();
        transferArm.execute();
        rotGripperArm.execute();

        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("sample state", pickUpSample);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("specStart", specimenStart);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        hardware = new Hardware(hardwareMap);
        sliderDev = new SliderDev(hardware);
        gripperArm = new ArmDev( hardware.gripperServo, ConfigVar.ArmCfg.GRIPPER_SPEED);
        handlerArm = new ArmDev(hardware.handlerServo, ConfigVar.ArmCfg.HANDLER_SPEED);
        poleArm1 = new ArmDev(hardware.poleServo1, ConfigVar.ArmCfg.POLE_SPEED );
        transferArm = new ArmDev(hardware.transferServo, ConfigVar.ArmCfg.TRANSFER_SPEED);
        turnerArm = new ArmDev(hardware.turnerServo, ConfigVar.ArmCfg.TURNER_SPEED);
        poleArm2 = new ArmDev(hardware.poleServo2, ConfigVar.ArmCfg.POLE_SPEED);
        rotGripperArm = new ArmDev(hardware.rotateGripper,ConfigVar.ArmCfg.ROT_GRIPPER_SPEED);

        setHomePositions();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        pathState = 0;

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

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
        rotGripperArm.moveTo( 150 );

        sliderDev.moveTo(ConfigVar.Slider.MIN_HEIGHT);

    }

}

