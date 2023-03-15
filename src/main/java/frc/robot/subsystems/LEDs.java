package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class LEDs extends SubsystemBase {
    private CANdle leds = new CANdle(22);
    private CANdleConfiguration ledConfig;
    private int orangeSwirlPosition = 0;
    private int blueSwirlPosition = (LedConstants.LED_LENGTH / 2) + 1;
    private String pieceType = "Cone";

    public LEDs() {
        ledConfig = new CANdleConfiguration();
        ledConfig.stripType = LedConstants.STRIP_TYPE;
        ledConfig.brightnessScalar = LedConstants.BRIGHTNESS;
        ledConfig.statusLedOffWhenActive = true;
        ledConfig.disableWhenLOS = false;
        ledConfig.vBatOutputMode = VBatOutputMode.Modulated;
        leds.configAllSettings(ledConfig);
        off();
    }

    @Override
    public void periodic() {
        // swirl();
        setColor(Color.kBeige);

        pieceType = LightningShuffleboard.getString("Leds", "Piece type", "None");
        hasPiece();
    }
    // Maybe add updating loop?

    /**
     * Blinks the leds between green and the color of the specified piece
     * needs to be called in a loop
     * 
     * @param pieceType the piece type
     */
    public void hasPiece() {
        // setColor(Color.kBeige);
        if (pieceType.equalsIgnoreCase("None")) { // TODO look at how we are calling this periodically only on button
                                                  // press?
            setColor(Color.kBlue);
        } else {
            if ((System.currentTimeMillis() % 1000) < 500) {
                setColor(Color.kGreen);
            } else {
                setColor(pieceType.equalsIgnoreCase("Cube") ? Color.kPurple : Color.kYellow);
            }
        }
    }

    public void wantsPiece() {
        setColor(pieceType.equalsIgnoreCase("WantsCube") ? Color.kPurple : Color.kYellow);
    }

    /**
     * Swirls orange and blue
     */
    public void swirl() { // TODO: make a way to periodically update the swirl
        // swirl pattern wooooooo (swirls orange and blue)
        if (System.currentTimeMillis() % 1000 == 0) {
            leds.setLEDs(0, 0, 255);
            for (int i = 0; i < (LedConstants.LED_LENGTH / 2); i++) {
                setColor(Color.kOrange, ((i + orangeSwirlPosition) % LedConstants.LED_LENGTH), 1);
            }
            for (int i = 0; i < (LedConstants.LED_LENGTH / 2); i++) {
                setColor(Color.kBlue, ((i + blueSwirlPosition) % LedConstants.LED_LENGTH), 1);

            }
            orangeSwirlPosition += 1;
            blueSwirlPosition += 1;
        }
    }

    /**
     * Turns off leds
     */
    public void off() {
        setColor(Color.kBlack);
    }

    /**
     * Sets the color of the LEDs using a color
     * 
     * @param color the color input
     */
    public void setColor(Color color) {
        leds.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
    }

    /**
     * Sets the color of the LEDs using a color
     * 
     * @param color    the color input
     * @param start    starting index to update
     * @param numberOf number of LEDs to update
     */
    public void setColor(Color color, int start, int numberOf) {
        leds.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, start, numberOf);
    }
}
