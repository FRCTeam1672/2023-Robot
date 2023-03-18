package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera photonCamera = new PhotonCamera("trashcan");


    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }
    public double getDistanceFromSubstation(){
        for (PhotonTrackedTarget target : photonCamera.getLatestResult().targets) {
            if(target.getFiducialId() == 4 || target.getFiducialId() == 8){
                return target.getBestCameraToTarget().getZ();
            }
        }
        return -1;
    }

    
}
