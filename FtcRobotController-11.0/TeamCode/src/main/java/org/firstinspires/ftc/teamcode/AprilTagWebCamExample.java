package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagWebCamExample  extends OpMode {

    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    @Override
    public void init() {
        aprilTagWebCam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebCam.update();
        AprilTagDetection id20 = aprilTagWebCam.getTagBySpecificId(20);
        telemetry.addData("id20String", id20.toString());
    }
}
