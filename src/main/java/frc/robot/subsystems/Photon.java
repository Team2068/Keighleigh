package frc.robot.subsystems;

import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;

public class Photon extends SubsystemBase {
    public PhotonCamera camera;
    public PhotonPipelineResult targetData;

    public double distance;
    public double xOffset;
    public double yOffset;

    public Photon(String net_name) {
        camera = new PhotonCamera(net_name);
    }

    public double getDistance() {
        double a2 = targetData.getBestTarget().getPitch();
        double a1 = LimelightConstants.LIMELIGHT_ANGLE;

        double result = 261.62 - LimelightConstants.LIMELIGHT_HEIGHT;
        double radians = Math.toRadians(a1 + a2);
        // double distance = result / Math.tan(radians);

        return Math.abs(result / Math.tan(radians)); // would return negative values if the angle was negative
    }

    public double calcRPM() {
        double distance = getDistance();
        double[] distTab = ShooterConstants.distTable;
        double[] rpmTab = ShooterConstants.rpmTable;

        int low = 0;
        int high = 0;

        for (int i = 0; i < distTab.length >> 1; i++) {
            if (distance < distTab[i]) { // If lower > dist -> upper bound found
                high = i;
                low = i - 1;
                break;
            }

            if (distance > distTab[distTab.length - i - 1]) { // If higher < dist -> higer is lower bound
                low = distTab.length - i - 1;
                high = low + 1;
                break;
            }
        }

        if (low == -1) return (distance * rpmTab[high]) / distTab[high];

        if (high >= distTab.length) return (distance * rpmTab[low]) / distTab[low];

        return rpmTab[low] + (distance - distTab[low]) * ((rpmTab[high] - rpmTab[low]) / (distTab[high] - distTab[low]));
    }

    public void ToggleCamMode() {
        camera.setDriverMode(!camera.getDriverMode());
    }

    public void SwitchPipeline() {
        int idx = camera.getPipelineIndex();

        if (idx < LimelightConstants.Pipelines.FIDICIAL)
            camera.setPipelineIndex(idx + 1);
        else
            camera.setPipelineIndex(0);
    }

    @Override
    public void periodic() {
        targetData = camera.getLatestResult();

        distance = getDistance(); // Upload to Robot State
        xOffset = targetData.getBestTarget().getYaw();
        yOffset = targetData.getBestTarget().getPitch();
    }
}
