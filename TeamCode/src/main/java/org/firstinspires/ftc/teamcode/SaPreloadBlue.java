package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
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

@Autonomous(name = "Preload Sample Blue")
public class SaPreloadBlue extends OpMode {
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

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 66   , Math.toRadians(90));


    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose preScorePose = new Pose(30, 30 , Math.toRadians(100));
    private final Pose scorePose = new Pose(28,28, Math.toRadians(100));
    private final Pose backOffPose = new Pose(28,35, Math.toRadians(100));

    private final Pose postScorePose = new Pose(24, 76.8, Math.toRadians(180));
    private final Pose fineScorePos = new Pose(38, 76.6, Math.toRadians(180));

    private final Pose pickUp1pos = new Pose(46, 35, Math.toRadians(270));
    private final Pose controlPickUp1pos = new Pose(19.212, 52.2);



    private Path preScorePreload;
    private Path backOffFromBasket;
    private Path scorePreload;

    public void buildPaths() {

        preScorePreload = new Path(new BezierLine(new Point(startPose), new Point(preScorePose)));
        preScorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading());

        scorePreload = new Path(new BezierLine(new Point(preScorePose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(preScorePose.getHeading(), scorePose.getHeading());

        backOffFromBasket = new Path(new BezierLine(new Point(scorePose), new Point(backOffPose)));
        backOffFromBasket.setLinearHeadingInterpolation(scorePose.getHeading(), backOffPose.getHeading());

    }


    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preScorePreload);
                gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
                pathState = 1;
                break;
            case 1:
                if(follower.isBusy())break;

                sliderDev.moveTo(ConfigVar.Slider.TOP_BASCKET_POS);
                gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
                pathState=2;
                break;
            case 2:
                if(sliderDev.notReady()) break;
                follower.followPath(scorePreload);
                gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
                pathState = 3;
                break;
            case 3:
                if(follower.isBusy()) break;

                gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
                pathState=4;
                break;
            case 4:
                if(gripperArm.notReady()) break;
                follower.followPath(backOffFromBasket);
                pathState= 5;
                break;
            case 5:
                if(follower.isBusy()) break;

                poleArm1.moveTo(ConfigVar.ArmCfg.poleIdle);
                poleArm2.moveTo(ConfigVar.ArmCfg.poleIdle);
                sliderDev.moveTo(0);



        }
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        poseUpdater.update();
        dashboardPoseTracker.update();

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

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

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

        setHomePositions();

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
        gripperArm.moveTo( ConfigVar.ArmCfg.gripperClosed);

        handlerArm.setRange(ConfigVar.ArmCfg.HANDLER_MIN, ConfigVar.ArmCfg.HANDLER_MAX);
        handlerArm.moveTo( ConfigVar.ArmCfg.handlerClosed);

        transferArm.setRange(ConfigVar.ArmCfg.TRANSFER_MIN, ConfigVar.ArmCfg.TRANSFER_MAX);
        transferArm.moveTo( ConfigVar.ArmCfg.transferSpPreCoop);

        poleArm1.setRange(ConfigVar.ArmCfg.POLE_MIN1,ConfigVar.ArmCfg.POLE_MAX1);
        poleArm1.moveTo( ConfigVar.ArmCfg.poleSaPlace2);

        poleArm2.setRange(ConfigVar.ArmCfg.POLE_MIN2,ConfigVar.ArmCfg.POLE_MAX2);
        poleArm2.moveTo( ConfigVar.ArmCfg.poleSaPlace2);

        turnerArm.setRange(ConfigVar.ArmCfg.TURNER_MIN,ConfigVar.ArmCfg.TURNER_MAX);
        turnerArm.moveTo( ConfigVar.ArmCfg.turnerIdle);

        rotGripperArm.setRange( ConfigVar.ArmCfg.ROT_GRIPPER_MIN,ConfigVar.ArmCfg.ROT_GRIPPER_MAX);
        rotGripperArm.moveTo( 150 );

        sliderDev.moveTo(ConfigVar.Slider.MIN_HEIGHT);

    }

}

