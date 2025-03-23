package frc.robot.subsystems;

import static frc.robot.Constants.StatusLEDConstants.statusLEDCount;

import java.time.Instant;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatusLED extends SubsystemBase {
    private static StatusLED instance;

    private final AddressableLED status_strip = new AddressableLED(9);
    AddressableLEDBuffer buf = new AddressableLEDBuffer(statusLEDCount);

    private StatusLED() {
        status_strip.setLength(statusLEDCount);

        status_strip.setData(buf);

        status_strip.start();

        setDefaultCommand(rgbStream());
    }

    public Command rgbStream() {
        return run(() -> {
            long time = System.currentTimeMillis() / 100;
            for (int i = 0; i < statusLEDCount; i++)
                buf.setHSV(i, (int) ((i * 2 + time) % 180), 255, 255);

            status_strip.setData(buf);
        }).ignoringDisable(true);
    }

    public Command flashColor(Color color, double rate) {
        return run(() -> {
            double time = ((double) System.currentTimeMillis()) / 1000;
            for (int i = 0; i < statusLEDCount; i++) {
                if ((time / rate) % 2 < 1)
                    // odds
                    if ((i & 1) == 1)
                        buf.setLED(i, color);
                    else
                        buf.setLED(i, Color.kWhite);
                else
                    // evens
                    if ((i & 1) == 1)
                        buf.setLED(i, Color.kWhite);
                    else
                        buf.setLED(i, color);
            }

            status_strip.setData(buf);
        }).ignoringDisable(true);
    }

    public static StatusLED getInstance() {
        if (instance == null) {
            instance = new StatusLED();
        }

        return instance;
    }
}
