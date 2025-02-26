package pedroPathing.tuners_tests.encoder_telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous
public class encodertest extends LinearOpMode {

    Hardware hw;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new Hardware(hardwareMap);

        hw.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hw.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hw.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hw.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        while (opModeIsActive()){

            telemetry.addData("rightFront", hw.rightFront.getCurrentPosition());
            telemetry.addData("leftBack", hw.leftBack.getCurrentPosition());
            telemetry.addData("rightBack", hw.rightBack.getCurrentPosition());
            telemetry.addData("leftFront", hw.leftFront.getCurrentPosition());
            telemetry.update();

        }
    }
}
