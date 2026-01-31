package org.firstinspires.ftc.teamcode.vision;
import org.firstinspires.ftc.teamcode.mechanisms.KitTurretControl;

public class TurretAutoAim {

    private KitTurretControl turret;
    private Limelight limelight;
    private double kP_pitch;
    private final int DESIRED_TAG_ID; // the specific tag to track

    // Constructor
    public TurretAutoAim(KitTurretControl turret, Limelight limelight, double kP_pitch, int desiredTagID) {
        this.turret = turret;
        this.limelight = limelight;
        this.kP_pitch = kP_pitch;
        this.DESIRED_TAG_ID = desiredTagID;
    }

     //Call this in your loop to auto-align turret pitch
     //@param autoAlignEnabled true if driver wants auto mode
     //@param manualInput joystick input for manual pitch fallback
    public void update(boolean autoAlignEnabled, double manualInput) {
        Integer targetID = limelight.getTargetID();

        if(autoAlignEnabled && targetID != null && targetID == DESIRED_TAG_ID) {
            double ty = limelight.getTY();
            double correction = -kP_pitch * ty; // may invert depending on servo setup
            turret.turretPitch(correction);
        } else {
            turret.turretPitch(manualInput);
        }
    }
}
