package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Random;

public class LedData {
    private I2C ledController;

    private static LedData instance = new LedData(52);

    private Random rand = new Random();

    public enum LedMode {
        SHOOT,
        TARGET,
        IDLE,
        CLIMB
      }

    public LedData(int ArduinoAddress) {
        this.ledController = new I2C(I2C.Port.kOnboard, ArduinoAddress);
    }

    public void startPattern(LedMode pattern) {
        byte anim;
        switch (pattern) {
            case SHOOT:
                anim = 0;
                break;
            case TARGET:
                anim = 1;
                break;
            case IDLE:
                anim = (byte)(rand.nextInt(2) + 2);
                break;
            case CLIMB:
                anim = 4;
                break;
            default:
                anim = (byte)(rand.nextInt(2) + 2);
                break;
        }
        byte[] data = {anim};
        this.ledController.writeBulk(data);
    }

    public void setAlliance() {
        DriverStation.Alliance alliance = DriverStation.getAlliance();
        int allianceId;
        if (alliance == DriverStation.Alliance.Red) {
            allianceId = 255;
        } else if (alliance == DriverStation.Alliance.Blue) {
            allianceId = 254;
        } else {
            allianceId = 253;
        }
        byte[] data = {(byte) allianceId};
        this.ledController.writeBulk(data);
    }

    public static LedData getInstance() {
        return instance;
    }
}
//LedData.getInstance().startPattern(SHOOT : TARGET : IDLE : CLIMB);