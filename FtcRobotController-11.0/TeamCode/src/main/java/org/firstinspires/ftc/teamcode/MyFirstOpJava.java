package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Field Centric TeleOp")
public class MyFirstOpJava extends OpMode {

    //Odometry Gobilda
    GoBildaPinpointDriver odo;

    // Motores
    private DcMotor BLeft;
    private DcMotor BRigth;
    private DcMotor FLeft;
    private DcMotor FRigth;

    @Override
    public void init() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        FLeft  = hardwareMap.get(DcMotor.class, "mfl");
        FRigth = hardwareMap.get(DcMotor.class, "mfr");
        BLeft  = hardwareMap.get(DcMotor.class, "mbl");
        BRigth = hardwareMap.get(DcMotor.class, "mbr");

        // Reverse motors

        //Odometry Configuration
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);  // ← CORREGIDO
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Starting position
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(
                DistanceUnit.MM , -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM)); // ← CORREGIDO
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM)); // ← CORREGIDO
        telemetry.addData("Device Version", odo.getDeviceVersion());
        telemetry.addData("Yaw Scalar", odo.getYawScalar());
        telemetry.update();
    }


    public void moveRobot(){
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2) - heading);
        double sinAngle = Math.sin((Math.PI / 2) - heading);

        double globalStrafe  = -forward * sinAngle + strafe * cosAngle;
        double globalForward =  forward * cosAngle + strafe * sinAngle;

        double fl = globalForward + globalStrafe + rotate;
        double fr = globalForward - globalStrafe - rotate;
        double bl = globalForward - globalStrafe + rotate;
        double br = globalForward + globalStrafe - rotate;

        // ← CORREGIDO (antes estabas usando FLeft para los 4 motores)
        FLeft.setPower(fl);
        FRigth.setPower(fr);
        BLeft.setPower(bl);
        BRigth.setPower(br);

        telemetry.addData("Robot X", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot Y", pos.getY(DistanceUnit.MM));
        telemetry.addData("Heading (rad)", heading);
        telemetry.update();
    }

    @Override
    public void loop() {
        moveRobot();

        telemetry.addData("Robot X", odo.getPosX(DistanceUnit.MM));  // ← CORREGIDO
        telemetry.addData("Robot Y", odo.getPosY(DistanceUnit.MM));  // ← CORREGIDO
        telemetry.addData("Heading", odo.getPosition().getHeading(AngleUnit.DEGREES));
        odo.update();
    }
}
