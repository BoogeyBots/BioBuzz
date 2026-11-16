package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.teamcode.Module.PinpointModule.distanta;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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
    public static double kp = 0.001, kp_recovery = 0.003, ki = 0.0, kd = 0.0, ks = 0.09, kv = 0.000496265, ka = 0.005, val_random = 1180, progresie = 4, regresie = 0.05;
    public static double vel20_25 = 1280, vel25_30 = 1300, vel30_35 = 1320, vel35_40 = 1320, vel40_45 = 1360, vel45_50 = 1350, vel50_55 = 1360;
    public static double ramp20_25 = 0.875, ramp25_30 = 0.85, ramp30_35 = 0.95, ramp35_40 = 0.88, ramp40_45 = 0.84, ramp45_50 = 0.81, ramp50_55 = 0.79;
    // Regresie rampa (valori intre -1 si 1, cu + sau -) adaugata la pozitia rampei dupa ce scade viteza (pt ultimele bile)
    public static double reg20_25 = 0.05, reg25_30 = 0.05, reg30_35 = 0.06, reg35_40 = 0.08, reg40_45 = 0.07, reg45_50 = 0.06, reg50_55 = 0.06;
    public static double target_velocity, rampa = 1;
    public static double velocity;
    public static boolean act_outtake;
    public static boolean rampa_jos = false;
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

        if (distanta <= 25 && act_outtake) {
            target_velocity = vel20_25;
            rampa = ramp20_25 + (rampa_jos ? reg20_25 : 0);
            servo_rampa.setPosition(Range.clip(rampa, 0.0, 1.0));
        }

        if (distanta <= 30 && distanta > 25 && act_outtake) {
            target_velocity = vel25_30;
            rampa = ramp25_30 + (rampa_jos ? reg25_30 : 0);
            servo_rampa.setPosition(Range.clip(rampa, 0.0, 1.0));
        }

        if (distanta <= 35 && distanta > 30 && act_outtake) {
            target_velocity = vel30_35;
            rampa = ramp30_35 + (rampa_jos ? reg30_35 : 0);
            servo_rampa.setPosition(Range.clip(rampa, 0.0, 1.0));
        }

        if (distanta <= 40 && distanta > 35 && act_outtake) {
            target_velocity = vel35_40;
            rampa = ramp35_40 + (rampa_jos ? reg35_40 : 0);
            servo_rampa.setPosition(Range.clip(rampa, 0.0, 1.0));
        }

        if (distanta <= 45 && distanta > 40 && act_outtake) {
            target_velocity = vel40_45;
            rampa = ramp40_45 + (rampa_jos ? reg40_45 : 0);
            servo_rampa.setPosition(Range.clip(rampa, 0.0, 1.0));
        }

        if (distanta <= 50 && distanta > 45 && act_outtake) {
            target_velocity = vel45_50;
            rampa = ramp45_50 + (rampa_jos ? reg45_50 : 0);
            servo_rampa.setPosition(Range.clip(rampa, 0.0, 1.0));
        }

        if (distanta <= 55 && distanta > 50 && act_outtake) {
            target_velocity = vel50_55;
            rampa = ramp50_55 + (rampa_jos ? reg50_55 : 0);
            servo_rampa.setPosition(Range.clip(rampa, 0.0, 1.0));
        }

        if (distanta > 55 && act_outtake) {
            double extra_power = distanta - 55;
            rampa = ramp50_55 + (rampa_jos ? reg50_55 : 0);
            servo_rampa.setPosition(Range.clip(rampa, 0.0, 1.0));
            target_velocity = vel50_55 + extra_power * progresie;
        }

        if (!act_outtake) {
            target_velocity = 1200;
        }

        // Folosim kp_recovery cand viteza a scazut sub tinta cu > 25 ticks/s (recuperare rapida intre cele 4 bile)
        double activeKp = (act_outtake && (target_velocity - velocity > 25)) ? kp_recovery : kp;
        controller.setPIDF(activeKp, 0, 0, 0);
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
