package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.teamcode.Module.TurretModule.target_x;
import static org.firstinspires.ftc.teamcode.Module.TurretModule.target_y;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointModule {
    HardwareMap hardwareMap;
    public double deltaX, deltaY;
    public static double distanta, currentHeading, currentX, currentY;
    public PinpointModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    GoBildaPinpointDriver pinpoint;

    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        pinpoint.setOffsets(2.767801960622232, -3.7192796722171817, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 0));
    }

    public void update() {
        pinpoint.update();

        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        currentX = pinpoint.getPosX(DistanceUnit.INCH);
        currentY = pinpoint.getPosY(DistanceUnit.INCH);

        deltaX = target_x - currentX;
        deltaY = target_y - currentY;

        distanta = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public void reset() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }
}