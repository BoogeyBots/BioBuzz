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

    // Puncte pentru update_interp, pornite din formulele actuale: vel = 1180 + 4d, rampa = 1 - 0.005d
    // Tragem între ~25 și ~50; 20 și 55 sunt doar margine
    public static double[] DIST = { 20,   25,    30,   35,    40,   45,    50,   55 };
    public static double[] VEL  = { 1260, 1280,  1300, 1320,  1340, 1360,  1380, 1400 };
    public static double[] RAMP = { 0.90, 0.875, 0.85, 0.825, 0.80, 0.775, 0.75, 0.725 };
    // 0.05 / 45 -> aceeași corecție ca treapta RAMPA_JOS la pragul PRAG_COBORARE
    public static double kRampComp = 0.0011, ramp_comp_max = 0.08;

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

    // drop = cât a scăzut viteza față de cea dinainte de tragere (0 când nu se trage)
    public void update_interp(double voltage, double drop) {
        velocity = -motorDR.getVelocity();
        if (act_outtake) {
            target_velocity = interp(DIST, VEL, distanta);

            double comp = Math.min(ramp_comp_max, kRampComp * Math.max(0, drop));
            rampa = interp(DIST, RAMP, distanta) + comp;

            servo_rampa.setPosition(rampa);
        }

        else {
            target_velocity = 1200;
        }
        controller.setPIDF(kp, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        double PID_output = controller.calculate(velocity, target_velocity);
        double ff_output = feedforward.calculate(target_velocity);
        double output = PID_output + ff_output;

        motorDR.setPower(output * (nominalvoltage / voltage));
        motorST.setPower(output * (nominalvoltage / voltage));
    }

    static double interp(double[] xs, double[] ys, double x) {
        if (x <= xs[0]) return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
        int i = 1;
        while (x > xs[i]) i++;
        double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
        return ys[i - 1] + t * (ys[i] - ys[i - 1]);
    }
}
