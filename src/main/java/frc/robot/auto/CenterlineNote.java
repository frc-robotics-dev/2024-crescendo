package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import lombok.Getter;

public enum CenterlineNote {
    A(FieldConstants.CenterlineNotes.ampSide),
    B(FieldConstants.CenterlineNotes.ampMid),
    C(FieldConstants.CenterlineNotes.center),
    D(FieldConstants.CenterlineNotes.sourceMid),
    E(FieldConstants.CenterlineNotes.sourceSide);

    @Getter private Translation2d translation;

    private CenterlineNote(Translation2d translation) {
        this.translation = translation;
    }
}