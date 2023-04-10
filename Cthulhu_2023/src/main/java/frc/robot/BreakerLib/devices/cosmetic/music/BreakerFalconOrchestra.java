// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.music;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/** Play audio via the speakers on CTRE Falcon 500 motors. */
public class BreakerFalconOrchestra extends SubsystemBase {

    private Orchestra orchestra;
    private String curSong;
    private boolean loopSong, playingMusic;

    private String[] playlist;
    private int index;
    private boolean loopAtEnd, playAutomatically;

    /** Creates an empty Falcon orchestra. */
    public BreakerFalconOrchestra() {
        orchestra = new Orchestra();
    }

    /**
     * Creates an Orchestra with the provided Falcons.
     * 
     * @param motors Falcon motors to include in the orchestra.
     */
    public BreakerFalconOrchestra(TalonFX... motors) {
        orchestra = new Orchestra();
        addOrchestraMotors(motors);
    }

    /**
     * @return Song playtime in milliseconds. If the song was stopped/never played,
     *         this value will be 0.
     */
    public int getPlaytimeMS() {
        return orchestra.getCurrentTime();
    }

    public boolean isPlaying() {
        return orchestra.isPlaying();
    }

    public boolean isPaused() {
        return !isPlaying() && getPlaytimeMS() != 0;
    }

    public boolean isStopped() {
        return !isPlaying() && getPlaytimeMS() == 0;
    }

    public boolean awaitingSong() {
        return isStopped() && playingMusic;
    }

    /**
     * Adds motors to the already-created Orchestra.
     * 
     * @param motors Falcon motors to add.
     */
    public void addOrchestraMotors(TalonFX... motors) {
        for (TalonFX motor : motors) {
            orchestra.addInstrument(motor);
        }
    }

    public void clearOrchestraMotors() {
        orchestra.clearInstruments();
    }

    public void play() {
        playingMusic = true;
        orchestra.play();
    }

    public void loadSong(String songPath) {
        curSong = songPath;
        orchestra.loadMusic(songPath);
    }

    public void pause() {
        playingMusic = false;
        orchestra.pause();
    }

    public void stop() {
        playingMusic = false;
        orchestra.stop();
    }

    public void setLooping(boolean loop) {
        loopSong = loop;
    }

    /**
     * Plays the given song.
     * 
     * @param songPath Path to the Chirp file on the RoboRIO. Files must be within
     *                 the deploy directory of the robot project and have the
     *                 extension .chrp.
     */
    public void playSong(String songPath) {
        loadSong(songPath);
        play();
    }

    public void loopSong(String songPath) {
        setLooping(true);
        playSong(songPath);
    }

    // PLAYLIST

    public void loadPlaylist(String... playlist) {
        this.playlist = playlist;
    }

    public void startPlaylistSong(int index) {
        try {
            this.index = index;
            curSong = playlist[index];
            playSong(curSong);
        } catch (IndexOutOfBoundsException e) {
            if (loopAtEnd) {
                startPlaylistSong(makeLoopIndex(index));
            } else {
                BreakerLog.log("Invalid song index.");
            }
        }
    }

    public void startPlaylist() {
        startPlaylistSong(0);
    }

    public void playNextSong() {
        startPlaylistSong(index + 1);
    }

    public void playPreviousSong() {
        startPlaylistSong(index - 1);
    }

    private int makeLoopIndex(int index) {
        int len = playlist.length;
        return index < 0 ? len - 1 : index % len;
    }

    @Override
    public void periodic() {
        // Loops song if song should loop.
        if (awaitingSong() && loopSong) {
            playSong(curSong);
        }
        // Queues next song if next song should be queued.
        else if (playAutomatically) {
            playNextSong();
        } 
    }
}
