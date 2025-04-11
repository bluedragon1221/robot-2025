package frc.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OverridableBeambreak {
    private boolean override;
    private DigitalInput beam_break;

    public OverridableBeambreak(int dio_pin) {
        beam_break = new DigitalInput(dio_pin);
    }

    public void setOverride(boolean override) {
        this.override = override;
    }

    public void toggleOverride() {
        override = !override;
    }

    public boolean getBool() {
        return beam_break.get() || override;
    }

    public Trigger isBroken() {
        return new Trigger(this::getBool);
    }
}
