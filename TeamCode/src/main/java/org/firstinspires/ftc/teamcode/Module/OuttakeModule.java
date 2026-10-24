package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.teamcode.Module.PinpointModule.distanta;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

@Configurable
public class OuttakeModule {
    HardwareMap hardwareMap;

    public OuttakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotorEx motorDR, motorST;
    Servo servo_rampa, servo_blocaj;
    public static double vel50_60 = 960, vel60_70 = 970, vel70_75 = 1045, vel75_80 = 1140, vel80_86 = 1200, vel86_95 = 1180, vel95_105 = 1250, vel105_116 = 1320, vel120 = 1450, progresie = 4.8; //1060 1090
    public static double kp = 0.003, ks = 0.1, kv = 0.000429266, ka = 0.005;
    public static double target_velocity, velocity;
    public static boolean act_outtake;
    public static double voltage;

    PIDFController controller = new PIDFController(kp, 0, 0, 0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

    public void init() {
        motorDR = hardwareMap.get(DcMotorEx.class, "dreapta");
        motorST = hardwareMap.get(DcMotorEx.class, "stanga");

        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");
        servo_rampa = hardwareMap.get(Servo.class, "servo_shooter");

        motorDR.setDirection(DcMotorSimple.Direction.REVERSE);

        blocat();
        servo_rampa.setPosition(0.28);

        target_velocity = 0;
    }
    public void blocat() {
        servo_blocaj.setPosition(0.02);
    }

    public void deblocat() {
        servo_blocaj.setPosition(0.3);
    }


    public void update(double vel) {
        velocity = motorDR.getVelocity();

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        controller.setPIDF(kp, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        if (act_outtake) {
            target_velocity = vel;
        }

        else {
            target_velocity = 1100;
        }

        double output = controller.calculate(velocity, target_velocity) + feedforward.calculate(target_velocity);

        motorDR.setPower(output * (9.3 / voltage));
        motorST.setPower(output * (9.3 / voltage));
    }
}