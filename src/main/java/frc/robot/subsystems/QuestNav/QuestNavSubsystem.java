package frc.robot.subsystems.QuestNav;

import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class QuestNavSubsystem extends SubsystemBase {
    QuestNav questNav = new QuestNav();

    private final DriveSubsystem driveSubsystem;

    public QuestNavSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    /**
     * This seems evil dont use :(
     * IF WE NEED THIS WE SHOULD FIX IT
     * It has no need to call "getAllUnreadPoseFrames" and we should make it not do that.
    */
    public Pose3d GetRobotPose3d() {
        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
        if (poseFrames.length > 0) {
            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();
            Pose3d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
            return robotPose;
        }
        return new Pose3d();
    }
    
    public void SetRobotPose3d(Pose3d pos) {
        Pose3d robotPose = pos;
        Pose3d questPose = robotPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST);
        questNav.setPose(questPose);
    }

    //The last published value to NetworkTables will persist even if QuestNav disconnects.
    //It is imperative you ensure the Quest is connected and is tracking before using its
    //pose data!

        Matrix<N3, N1> QUESTNAV_STD_DEVS = 
            VecBuilder.fill(
                0.02,
                0.02,
                0.035
            );
        
    @Override
    public void periodic() {
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
        
        for (PoseFrame questFrame : questFrames) {
            if (questFrame.isTracking()) {
                Pose3d questPose = questFrame.questPose3d();
                double timestamp = questFrame.dataTimestamp();
                Pose3d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
                driveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            }
        }
    }
}

