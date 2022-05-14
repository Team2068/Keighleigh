package frc.robot.subsystems;


import java.util.function.Consumer;

import com.kauailabs.navx.frc.AHRS;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;

public class Photon extends SubsystemBase{
    public PhotonCamera camera;
    public PhotonPipelineResult targetData;
    
    public double distance;
    public double horizontalOffset;
    public double verticalOffset;

    public Photon(String net_name){
        camera = new PhotonCamera(net_name);
    }

    public void adjustGyroDrift(AHRS navX){
        //TODO: Switch Pipeline to some sort of fidical pipeline
        double camOffset = targetData.getBestTarget().getYaw();
        double gyroAngle = navX.getYaw();

        // double gyro_estimate_x = (Math.cos(gyroAngle) * distance);
        // double gyro_estimate_y = Math.sin(gyroAngle) * distance;

        // double approximate_x = (Math.cos(camOffset) * distance);
        // double approximate_y = Math.sin(camOffset) * distance;

        double gyro_error_x = (Math.cos(gyroAngle) * distance) - (Math.cos(camOffset) * distance);
        double gyro_error_y = (Math.sin(gyroAngle) * distance) - (Math.sin(camOffset) * distance);

        if (gyro_error_x > 1 || gyro_error_y > 1) // 1 is placeholder margin of error 
            navX.setAngleAdjustment(-(Math.atan(gyro_error_y/gyro_error_x))); // Adjust the Gyro by the error to get back to the expected results
        //TODO: Switch Back to reflective tape pipline
    }

    public void adjustOdometry(Pose2d currentPose, Consumer<Pose2d> result){
        double camOffset = targetData.getBestTarget().getYaw();

        Translation2d camTrans = new Translation2d((Math.cos(camOffset) * distance), (Math.sin(camOffset) * distance));
        
        double odometry_err_x = currentPose.getX() - camTrans.getX();
        double odometry_err_y = currentPose.getY() - camTrans.getY();

        if (odometry_err_x > 1 || odometry_err_y > 1)
        result.accept(new Pose2d(camTrans, currentPose.getRotation()));
    }

    public double getDistance() {
        double a2 = targetData.getBestTarget().getPitch();
        double a1 = LimelightConstants.LIMELIGHT_ANGLE;
    
        double result = 261.62 - LimelightConstants.LIMELIGHT_HEIGHT;
        double radians = Math.toRadians(a1 + a2);
        // double distance = result / Math.tan(radians);
    
        return Math.abs(result / Math.tan(radians)); // would return negative values if the angle was negative
    }

    public double calcRPM(){
        double distance = getDistance();
        double[] distTab = ShooterConstants.distTable;
        double[] rpmTab = ShooterConstants.rpmTable;
    
        int low = 0;
        int high = 0;
    
        for (int i = 0; i < distTab.length >> 1; i++) {
          if (distance < distTab[i]){ //If lower > dist -> upper bound found
            high = i; 
            low = i-1;
            break;
          }
    
          if (distance > distTab[distTab.length - i - 1]){ //If higher < dist -> higer is lower bound
            low = distTab.length - i - 1;
            high = low + 1;
            break;
          }
        }
    
        if (low == -1 ) return (distance * rpmTab[high])/distTab[high];
        
        if (high >= distTab.length) return (distance * rpmTab[low])/distTab[low];
        
        return rpmTab[low] + (distance - distTab[low])*((rpmTab[high]- rpmTab[low])/(distTab[high]- distTab[low]));
    }

    @Override
    public void periodic(){
        targetData = camera.getLatestResult();

        distance = getDistance(); // Upload to Robot State
        horizontalOffset = targetData.getBestTarget().getYaw();
        verticalOffset = targetData.getBestTarget().getPitch();
    }

    // Specilized Functions
    public void ToggleCamMode(){
        camera.setDriverMode(!camera.getDriverMode());
    }

    public void SwitchPipeline(){
        int idx = camera.getPipelineIndex();

        if (idx < LimelightConstants.Pipelines.FIDICIAL)
            camera.setPipelineIndex(idx+1);
        else
            camera.setPipelineIndex(0);
    }
}