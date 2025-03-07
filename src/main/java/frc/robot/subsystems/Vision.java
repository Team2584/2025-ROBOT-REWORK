package frc.robot.subsystems;

import com.frcteam3255.utils.LimelightHelpers;
import com.frcteam3255.utils.LimelightHelpers.PoseEstimate;
import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_VISION;

@Logged
public class Vision extends SubsystemBase {
    @NotLogged
    PoseEstimate lastEstimateFront = new PoseEstimate();
    @NotLogged
    PoseEstimate lastEstimateBack = new PoseEstimate();

    @NotLogged
    boolean newFrontEstimate = false;
    @NotLogged
    boolean newBackEstimate = false;

    Pose2d rightPose = new Pose2d();
    Pose2d leftPose = new Pose2d();

    private boolean useMegaTag2 = true;

    public Vision() {
    }

    @NotLogged
    public PoseEstimate[] getLastPoseEstimates() {
        return new PoseEstimate[] { lastEstimateFront, lastEstimateBack };
    }

    public void setMegaTag2(boolean useMegaTag2) {
        this.useMegaTag2 = useMegaTag2;
    }

    /**
     * Determines if a given pose estimate should be rejected.
     * 
     * 
     * @param poseEstimate The pose estimate to check
     * @param gyroRate     The current rate of rotation observed by our gyro.
     * 
     * @return True if the estimate should be rejected
     */

    public boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate) {
        // Angular velocity is too high to have accurate vision
        if (gyroRate.compareTo(CONSTANTS_VISION.MAX_ANGULAR_VELOCITY) > 0) {
            return true;
        }

        // No tags :(((
        if (poseEstimate.tagCount == 0) {
            return true;
        }

        // 1 Tag with a large area
        if (poseEstimate.tagCount == 1 && poseEstimate.avgTagArea > CONSTANTS_VISION.AREA_THRESHOLD) {
            return false;
            // 2 tags or more
        } else if (poseEstimate.tagCount > 1) {
            return false;
        }

        return true;
    }

    /**
     * Updates the current pose estimates for the left and right of the robot using
     * data from Limelight cameras.
     *
     * @param gyroRate The current angular velocity of the robot, used to validate
     *                 the pose estimates.
     *
     *                 This method retrieves pose estimates from two Limelight
     *                 cameras (left and right) and updates the
     *                 corresponding pose estimates if they are valid. The method
     *                 supports two modes of operation:
     *                 one using MegaTag2 and one without. The appropriate pose
     *                 estimate retrieval method is chosen
     *                 based on the value of the `useMegaTag2` flag.
     *
     *                 If the retrieved pose estimates are valid and not rejected
     *                 based on the current angular velocity,
     *                 the method updates the last known estimates and sets flags
     *                 indicating new estimates are available.
     */
    public void setCurrentEstimates(AngularVelocity gyroRate) {
        PoseEstimate currentEstimateFront = new PoseEstimate();
        PoseEstimate currentEstimateBack = new PoseEstimate();

        if (useMegaTag2) {
            currentEstimateFront = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(CONSTANTS_VISION.LIMELIGHT_NAMES[0]);
            currentEstimateBack = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(CONSTANTS_VISION.LIMELIGHT_NAMES[1]);
        } else {
            currentEstimateFront = LimelightHelpers.getBotPoseEstimate_wpiBlue(CONSTANTS_VISION.LIMELIGHT_NAMES[0]);
            currentEstimateBack = LimelightHelpers.getBotPoseEstimate_wpiBlue(CONSTANTS_VISION.LIMELIGHT_NAMES[1]);
        }

        if (currentEstimateFront != null && !rejectUpdate(currentEstimateFront, gyroRate)) {
            lastEstimateFront = currentEstimateFront;
            rightPose = currentEstimateFront.pose;
            newFrontEstimate = true;
        }
        if (currentEstimateBack != null && !rejectUpdate(currentEstimateBack,
                gyroRate)) {
            lastEstimateBack = currentEstimateBack;
            leftPose = currentEstimateBack.pose;
            newBackEstimate = true;
        }
    }

    public Optional<PoseEstimate> determinePoseEstimate(AngularVelocity gyroRate) {
        setCurrentEstimates(gyroRate);

        // No valid pose estimates :(
        if (!newFrontEstimate && !newBackEstimate) {
            return Optional.empty();

        } else if (newFrontEstimate && !newBackEstimate) {
            // One valid pose estimate (right)
            newFrontEstimate = false;
            return Optional.of(lastEstimateFront);

        } else if (!newFrontEstimate && newBackEstimate) {
            // One valid pose estimate (left)
            newBackEstimate = false;
            return Optional.of(lastEstimateBack);

        } else {
            return Optional.of(lastEstimateFront);
        }
    }

    @Override
    public void periodic() {
    }
}
