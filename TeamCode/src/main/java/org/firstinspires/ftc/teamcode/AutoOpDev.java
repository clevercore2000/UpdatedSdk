package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous
public class AutoOpDev extends LinearOpMode
//DEBUGPC class  AutoOpDev
{
    /*DEBUGPC
    public static void main(String[] args)
    {
        AutoOpDev autoOpDev = new AutoOpDev();
        autoOpDev.runOpMode();
    }
     */
    /*
     *   - Manage the 30 sec autonomus timing
     *   - The main loop iteration ( or implement with threads ?? )
     *   - Controls connection
     *   - Telemetry connection
     *   -
     */
    private  ElapsedTime tm;            // Timer
    private  double lastOpDuration = 0; // Returns the time of autonomous run
    private  RCodeParser Parser;        // R-Code parser
    private Hardware hardware;
    private ArmDev   gripperArm;    // Gripper servo
    // g1GripperArm is the index in the Gxx function library
    // When a command block G1 in R-Code is parsed the code will move gripperArm servo
    private  String  g1GripperArm = "G1";  // Asocieate gripperArm with G1 in R-Code
    private  ArmDev  handlerArm;    // Handler servo
    // Define G2 in R-Code as GripperArm
    private  String  g2HandlerArm = "G2";  // Asocieate handlerArm with G2 in R-Code
    private  ArmDev  transferArm;   // Transfer servo
    private String           g3Transfer = "G3";    // Asociate transferArm with G3 in R-Code
    private  ArmDev  poleArm;       // Pole servo
    private  String  g4PoleArm = "G4";    // Asocieate poleArm with G4 in R-Code
    private  ArmDev  turnerArm;     // Turner servo
    private  String  g5TurnerArm = "G5";   // Asocieate turnerArm with G5 in R-Code
    private  SliderDev sliderDev;   // Slider
    private  MecanumDev mecanumDev; // Mecanum chassis
    boolean modalServoGroup = false;    // Servo Modal Group status
    boolean modalSliderGroup = false;   // Slider Modal Group status
    boolean modalMecanumGroup = false;// Mecanum Modal Group status
    // Status of R-Code execution
    private enum AutoRunStatus {blockStopped, blockRqParse, blockRunning, blockReqStop, blockModal }
    private AutoRunStatus  Status;
    private  String [] BlockWords;    // This array contains the command words from current R-Code block

    @Override
    public void runOpMode()
    {
        Initialize();
        waitForStart();
        tm.startTime();
        boolean opModeIsActive = true; // Replace with actual condition if available
        while(opModeIsActive)
        {


            mecanumDev.execute();   // Mecanum drive
            sliderDev.execute();    // Slider system
            gripperArm.execute();   // Gripper servo

            //handlerArm.execute();   // Handler servo NOT started up yet
            //transferArm.execute();  // Transfer servo NOT started up yet
            poleArm.execute();      // Pole servo
            //turnerArm.execute();    // Turner servo NOT started up yet
            execute();         // Processes the R-Code

/*
            telemetry.addData("Blk:", exeBlock);
            telemetry.addData("Ztg ", sliderDev.targetPos );
            telemetry.addData("Zact", sliderDev.actPos );
            telemetry.addData("Zerr", sliderDev.getDeviation() );
            telemetry.addData("Zspe", sliderDev.actSpeed );
            telemetry.update();
 */
        }
        stopOp();
    }

    public void Initialize()
    {
        try{
            // ... Instantiate parser and system variables ...
            hardware = new Hardware(hardwareMap);
            Parser = new RCodeParser(); // R-Code interpreter
            // Instantiate all robot objects
            mecanumDev = new MecanumDev(hardware);
            sliderDev = new SliderDev( hardware);
            gripperArm = new ArmDev( hardware.gripperServo, g1GripperArm, 200);
            handlerArm = new ArmDev( hardware.handlerServo, g2HandlerArm, 200);
            transferArm = new ArmDev(hardware.transferServo, g3Transfer, 200);
            poleArm = new ArmDev( hardware.poleServo, g4PoleArm, 200);
            tm = new ElapsedTime();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        // Initialize robot objects
        mecanumDev.Initialize();

        gripperArm.setRange(ConfigVar.ArmCfg.GRIPPER_MIN, ConfigVar.ArmCfg.GRIPPER_MAX);
        gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);

        handlerArm.setRange(ConfigVar.ArmCfg.HANDLER_MIN, ConfigVar.ArmCfg.HANDLER_MAX);
        transferArm.setRange(ConfigVar.ArmCfg.TRANSFER_MIN, ConfigVar.ArmCfg.TRANSFER_MAX);
        poleArm.setRange(ConfigVar.ArmCfg.POLE_MIN,ConfigVar.ArmCfg.POLE_MAX);
        Status = AutoRunStatus.blockStopped;
    }
    /*
     *   startOp
     *   ** Starts running the MAP
     *   ** It has the name "startOp" because method the named "start" starts the thread
     *
     */
    boolean startOp()
    {
        // If not in stop then ignore the request
        if( Status != AutoRunStatus.blockStopped)
            stopOp();
        // Go to set the current map step to the robot
        Status = AutoRunStatus.blockStopped;
        return true;
    }
    /*
     *   stopOp
     *   ** Stops running the MAP
     *   ** It has the name "stopOp" because method the named "stop" stops the thread
     *
     */
    public  void stopOp()
    {
        if(Status != AutoRunStatus.blockStopped)
        {
            //  Forced stop all systems
            mecanumDev.stop();
            sliderDev.stop();
            gripperArm.stop();
            handlerArm.stop();
            transferArm.stop();
            poleArm.stop();
            turnerArm.stop();
            // Set the robot in rcStopped status
            Status = AutoRunStatus.blockStopped;
        }
    }
    // Method: allSystemsAreStopped
    // returns true when all robot objects are stopped
    public  boolean allSystemsAreStopped()
    {
        // Check that all systems are in position or stopped
        return
                (
                mecanumDev.isReady()
                &&  sliderDev.isReady()
                && sliderDev.isReady()
                &&  gripperArm.isReady()
                &&  handlerArm.isReady()
                &&  transferArm.isReady()
                &&  poleArm.isReady()
                && turnerArm.isReady()
                );
    }

    boolean modalMGroup = false;

    public  void execute/*RCode*/()
    {
        // This switch controls the status of running R-Code blocks
        switch ( Status )
        {
            case blockStopped:
                // Start the AutoOpMode R-Code execution
                if( /*DEBUG gamepad1.dpad_up && */  ( Parser.rcStatus == RCodeParser.RCPrgStatus.rcEnded || Parser.rcStatus == RCodeParser.RCPrgStatus.rcNotLoaded ))
                {
                    Parser.loadFile();      // Load R-Code file (ex: "auto.rc")
                    startOp();
                    tm.reset();
                    Status = AutoRunStatus.blockRqParse;
                }
                break;
            case blockRqParse:
                // Get next Block of R-Code
                // If non in modal then execute block
                if( ! ( modalMecanumGroup || modalSliderGroup || modalServoGroup || modalMGroup ) )
                    if( !Parser.nextBlock() )
                    {
                        // Some error occurred while loading the program file
                        Status = AutoRunStatus.blockStopped;
                        break;
                    }
                // All command words in current block have been processed
                Status = AutoRunStatus.blockRunning;
                break;
            case blockRunning:
                // Process mecanum command words
                if( !Parser.mecanumModalActive( mecanumDev.isReady()) )
                    mecanumDev.moveTo(Parser.getTarget("X"), Parser.getTarget("Y"), Parser.getTarget("R"), Parser.getTarget("F"));
                // Process slider command words
                if( !Parser.sliderModalActive( sliderDev.isReady()) )
                    sliderDev.moveTo(Parser.getTarget("Z"), Parser.getTarget("S"));
                // Process Servos command words
                modalServosGroup();
                Status = ( modalMecanumGroup || modalSliderGroup || modalServoGroup || modalMGroup )? AutoRunStatus.blockRunning : AutoRunStatus.blockRqParse;
                // Process M,P,Q command words
                modalMGroup = false;
                modalMCodes();
                Parser.rcSetPoints.clear();
                if(Parser.isModalActive("M2") ) modalMGroup = true;
                // Add Sensor condition to disable MODAL status for P1
                if( Parser.modalFuncActive("P1", true/*Sensor Status to release Modal P1*/)) modalMGroup = true;
                // Add Sensor condition to disable MODAL status for P1
                if( Parser.modalFuncActive("Q1",true /* && Sensor-2 active */) ) modalMGroup = true;
                break;
            case blockReqStop:
                // Executes waits for systems to finish their moves
                if( allSystemsAreStopped() )
                {
                    // Stop all robot objects
                    stopOp();
                    lastOpDuration = tm.seconds();
                    // Set status to R-Code stopped
                    Status = AutoRunStatus.blockStopped;
                }
                break;
        }
    }
    // Method: runMCodes
    // Processes the Mx command blocks in R-Code
    public void modalMCodes()
    {
        // M99 - is R-Code end
        if( Parser.blockActive("M99")) Status = AutoRunStatus.blockReqStop;
        // Proccess M2 function ==>> wait for all systems to finish ops
        Parser.parseModal() ;   // Parses modal command words P1..P4, Q1..Q4, M2
        // Processes the M2 condition
        if( Parser.isModalActive("M2") && allSystemsAreStopped() ) Parser.disableModal("M2");
    }

    // If any servo is moving and and on a next block commes a new command word for any servo
    // then it goes MODAL and waits for the moving servo ( servo's) to finish move
    public void modalServosGroup()
    {
        boolean allServosReady = false;
        if( gripperArm.isReady() && handlerArm.isReady() && transferArm.isReady() && poleArm.isReady() && turnerArm.isReady() )
            allServosReady = true;
        modalServoGroup = Parser.servoModalActive( allServosReady);
    }
}
