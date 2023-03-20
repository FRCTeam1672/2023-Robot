package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BiConsumer;

public class LEDLightSubsystem extends SubsystemBase {
    private final AddressableLED addressableLED = new AddressableLED(7);
    private final AddressableLEDBuffer addressableLEDBuffer = new AddressableLEDBuffer(149);
    private LedState currentState = LedState.REDPULSE;
    private final PersistentLedState persistentLedState = new PersistentLedState();

    private static class PersistentLedState {
        public int rainbowFirstPixelHue = 0;
        public int pulseOffset = 0;
    }

    public enum LedState {
        RED((ledBuffer, persistentState) -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 255, 0, 0);
            }
        }),
        BLUE((ledBuffer, persistentState) -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 0, 0, 255);
            }
        }),
        PURPLE((ledBuffer, persistentState) -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 145, 0, 255);
            }
        }),
        YELLOW((ledBuffer, persistentState) -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 255, 225, 0);
            }
        }),
        REDPULSE((ledBuffer, persistentState) -> {
            int intensity = 255 - persistentState.pulseOffset;
            for (int i = 0; i < ledBuffer.getLength() / 2 + 1; i++) {
                ledBuffer.setRGB(i, (intensity + 25*i) % 255, 0, 0);
                ledBuffer.setRGB(ledBuffer.getLength()-1-i, (intensity + 25*i) % 255, 0, 0);
            }
            persistentState.pulseOffset = (persistentState.pulseOffset + 15) % 255;
        }),
        BLUEPULSE((ledBuffer, persistentState) -> {
            int intensity = 255 - persistentState.pulseOffset;
            for (int i = 0; i < ledBuffer.getLength() / 2 + 1; i++) {
                ledBuffer.setRGB(i, 0, 0, (intensity + 25*i) % 255);
                ledBuffer.setRGB(ledBuffer.getLength()-1-i, 0, 0, (intensity + 25*i) % 255);
            }
            persistentState.pulseOffset = (persistentState.pulseOffset + 15) % 255;
        }),
        RAINBOW((ledBuffer, persistentState) -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                final int hue = (persistentState.rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
                ledBuffer.setHSV(i, hue, 255, 128);
            }
            persistentState.rainbowFirstPixelHue += 3;
            persistentState.rainbowFirstPixelHue %= 180;
        }),
        OFF((ledBuffer, persistentState) -> {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        });

        public final BiConsumer<AddressableLEDBuffer, PersistentLedState> setBuffer;

        LedState(BiConsumer<AddressableLEDBuffer, PersistentLedState> bufferSetter) {
            setBuffer = bufferSetter;
        }
    }

    public LEDLightSubsystem() {
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    public void idle() {
        Alliance alliance = DriverStation.getAlliance();


        if(alliance == Alliance.Blue) {
            currentState = LedState.BLUEPULSE;
        } else if(alliance == Alliance.Red) {
            currentState = LedState.REDPULSE;
        } else {
            currentState = LedState.RAINBOW;
        }
    }
    public void setColor(LedState ledState){
        this.currentState = ledState;
    }

    @Override
    public void periodic() {
        if(!DriverStation.isEnabled()) idle();
        else if(currentState == LedState.PURPLE || currentState == LedState.YELLOW) {}
        else setColor(LedState.RAINBOW); 
        currentState.setBuffer.accept(addressableLEDBuffer, persistentLedState);
        addressableLED.setData(addressableLEDBuffer);
    }
}