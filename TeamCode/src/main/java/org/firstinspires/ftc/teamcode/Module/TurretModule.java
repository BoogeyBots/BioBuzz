package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.teamcode.Module.PinpointModule.currentHeading;
import static org.firstinspires.ftc.teamcode.Module.PinpointModule.currentX;
import static org.firstinspires.ftc.teamcode.Module.PinpointModule.currentY;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Configurable
public class TurretModule {
    HardwareMap hardwareMap;
    double GEAR_RATIO = (60.0 / 20.0) * (25.0 / 65.0);

    public TurretModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    Servo servo_right, servo_left;

    private static final double MIN_ANGLE = -160.0, MAX_ANGLE = 160.0;
    private static final double MAX_SERVO_STEP = 0.05;
    public static double decalation;
    public static double target_x = 85, target_y = 57;

    private double lastServoPos = 0.5;

    public void init() {
        servo_right = hardwareMap.get(Servo.class, "servo_right");
        servo_left = hardwareMap.get(Servo.class, "servo_left");

        decalation = 0;
    }


    public void update() {
        double target_angle = Math.toDegrees(Math.atan2(target_y - currentY, target_x - currentX));

        double robotRelative = AngleUnit.normalizeDegrees(target_angle - currentHeading);
        double turretAngle = AngleUnit.normalizeDegrees(robotRelative - 180.0);
        double clippedAngle = Range.clip(turretAngle, MIN_ANGLE, MAX_ANGLE);

        double requestedServoPos = 0.5 + ((clippedAngle * GEAR_RATIO) / 320) + decalation;

        double safeServoPosition = Range.clip(requestedServoPos,
                lastServoPos - MAX_SERVO_STEP,
                lastServoPos + MAX_SERVO_STEP);

        lastServoPos = safeServoPosition;

        servo_right.setPosition(safeServoPosition);
        servo_left.setPosition(safeServoPosition);
    }
}