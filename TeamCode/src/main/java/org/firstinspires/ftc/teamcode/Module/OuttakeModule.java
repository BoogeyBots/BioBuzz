package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.teamcode.Module.PinpointModule.distanta;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

@Configurable
public class OuttakeModule {
    private final HardwareMap hardwareMap;

    public OuttakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    private DcMotorEx motorDR, motorST;
    public Servo servo_rampa, servo_blocaj;
    public static double kp = 0.001, ki = 0.0, kd = 0.0, ks = 0.09, kv = 0.000496265, ka = 0.005, val_random = 1180, progresie = 4, regresie = 0.005;
    public static double target_velocity, rampa = 1;
    public static double velocity;
    public static boolean act_outtake;
    double nominalvoltage = 9.3;
    public static double rampa_reper = 0.96;

    private PIDFController controller;
    private SimpleMotorFeedforward feedforward;

    public void init() {
        motorDR = hardwareMap.get(DcMotorEx.class, "dreapta");
        motorST = hardwareMap.get(DcMotorEx.class, "stanga");

        servo_blocaj = hardwareMap.get(Servo.class, "servo_blocaj");
        servo_rampa = hardwareMap.get(Servo.class, "servo_shooter");

        motorDR.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(kp, ki, kd, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        blocat();
        target_velocity = 1200;
    }

    public void blocat() {
        servo_blocaj.setPosition(0.02);
    }

    public void deblocat() {
        servo_blocaj.setPosition(0.3);
    }

    public void update(double voltage) {
        velocity = -motorDR.getVelocity();
        if (act_outtake) {
            target_velocity = val_random + distanta * progresie;

            rampa = rampa_reper - distanta * regresie; //regresie - 0.0057

            servo_rampa.setPosition(rampa);
        }

        else {
            target_velocity = 1200;
        }
        controller.setPIDF(kp, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);


        if(target_velocity != 1200) {
            double PID_output = controller.calculate(velocity, target_velocity);
            double ff_output = feedforward.calculate(target_velocity);
            double output = PID_output + ff_output;

            motorDR.setPower(output * (nominalvoltage / voltage));
            motorST.setPower(output * (nominalvoltage / voltage));
        }

        else {
            double PID_output = controller.calculate(velocity, 1200);
            double ff_output = feedforward.calculate(1200);
            double output = PID_output + ff_output;
            motorDR.setPower(output * (nominalvoltage / voltage));
            motorST.setPower(output * (nominalvoltage / voltage));
        }
    }
}
