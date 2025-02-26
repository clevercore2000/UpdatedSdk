package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.DriveEncoderConstants;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {
    static {
        DriveEncoderConstants.forwardTicksToInches = 0.011;
        DriveEncoderConstants.strafeTicksToInches = 0.0117;
        DriveEncoderConstants.turnTicksToInches = 0.0117 ;

        DriveEncoderConstants.robot_Width = 14.173;
        DriveEncoderConstants.robot_Length = 13.228;

        DriveEncoderConstants.leftFrontEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.rightFrontEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.leftRearEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.rightRearEncoderDirection = Encoder.FORWARD;
    }
}




