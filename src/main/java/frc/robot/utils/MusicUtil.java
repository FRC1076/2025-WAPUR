package frc.robot.utils;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

/** Class for playing music on the Kraken Orchestra.
 * Calling any method in this class enables the Orchestra by instantiating the object
 * (waits for call to save memory) */
public final class MusicUtil {
    private static boolean enabled;
    private static Orchestra m_orchestra;

    /** Allow music to play */
    public static void enableMusic() {
        if (!enabled) {
            m_orchestra = new Orchestra();
            enabled = true;
        }
    }

    /** Add a motor to the orchestra. */
    public static StatusCode addInstrument(TalonFX device) {
        if (!enabled) {
            enableMusic();
        }
        return m_orchestra.addInstrument(device);
    }

    /** Load music into the orchestra. <p>
     *  <b>WARNING: This action is blocking, so do not call except while disabled.</b> 
     * 
     *  @param filePath The path of the file in the deploy directory.
     */
    public static StatusCode loadMusic(String filePath) {
        if (!enabled) {
            enableMusic();
        }
        return m_orchestra.loadMusic(filePath);
    }

    /** Play the current music. */
    public static StatusCode playMusic() {
        if (!enabled) {
            enableMusic();
        }
        return m_orchestra.play();
    }

    /** Pause the current music. */
    public static StatusCode pauseMusic() {
        if (!enabled) {
            enableMusic();
        }
        return m_orchestra.pause();
    }

    /** Stop the music. */
    public static StatusCode stopMusic() {
        if (!enabled) {
            enableMusic();
        }
        return m_orchestra.stop();
    }
}
