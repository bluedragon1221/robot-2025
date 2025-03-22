package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.StatusLEDConstants.*;

public class StatusLED extends SubsystemBase {
    private static StatusLED instance;

    private final AddressableLED status_strip = new AddressableLED(9);
    AddressableLEDBuffer buf = new AddressableLEDBuffer(statusLEDCount);
    double tick = 0;

    private StatusLED() {
        status_strip.setLength(statusLEDCount);

        status_strip.setData(buf);

        status_strip.start();
    }

    @Override
    public void periodic() {
        for (int i=0;i<statusLEDCount;i++)
            buf.setHSV(i, (i * 2 + (int) tick) % 180, 255, 255);

        tick += 1;
        
        status_strip.setData(buf);
    }

    public static StatusLED getInstance() {
        if (instance == null) {
            instance = new StatusLED();
        }

        return instance;
    }
}
