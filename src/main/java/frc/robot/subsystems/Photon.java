package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

public class Photon {
    final PhotonCamera camera;

    public Photon(String net_name){
        camera = new PhotonCamera(net_name);
    }
}
