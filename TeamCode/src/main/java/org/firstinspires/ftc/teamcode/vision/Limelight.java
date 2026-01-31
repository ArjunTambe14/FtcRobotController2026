package org.firstinspires.ftc.teamcode.vision;

import org.json.JSONObject;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

public class Limelight {
    private final String limelightIP;

    public Limelight(String ip) {
        this.limelightIP = ip;
    }

    // ------------------- Get vertical offset -------------------
    public double getTY() {
        return getValue("ty");
    }

    // ------------------- Check if any target visible -------------------
    public boolean hasTarget() {
        return getValue("tv") == 1.0;
    }

    // ------------------- Get AprilTag ID -------------------
    public Integer getTargetID() {
        if (!hasTarget()) return null;
        double tid = getValue("tid");  // read from JSON
        return (int) tid;
    }

    // ------------------- Internal helper -------------------
    private double getValue(String key) {
        try {
            URL url = new URL("http://" + limelightIP + ":5807/limelight/json");
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");

            BufferedReader reader = new BufferedReader(
                    new InputStreamReader(connection.getInputStream())
            );
            StringBuilder response = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) response.append(line);
            reader.close();

            JSONObject json = new JSONObject(response.toString());
            return json.optDouble(key, 0.0);  // default to 0 if key missing
        } catch (Exception e) {
            return 0.0; // fail-safe if Limelight isn’t reachable
        }
    }
}
