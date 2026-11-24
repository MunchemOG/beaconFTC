package org.firstinspires.ftc.teamcode.beacon.vision;

/**
 * Decodes one JPEG into 8-bit gray. The robot uses {@link AndroidJpegDecoder}; tests use a
 * desktop decoder. Called only from the frame source's decode thread.
 */
public interface JpegDecoder {
    /**
     * Decodes {@code jpeg[0, length)} into {@code out} (calls {@link GrayFrame#ensureSize} and
     * fills {@link GrayFrame#gray}). Only the pixel fields are touched.
     * @return false if the data is not a decodable image
     */
    boolean decode(byte[] jpeg, int length, GrayFrame out);
}
