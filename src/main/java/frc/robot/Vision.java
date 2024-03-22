package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.management.modelmbean.InvalidTargetObjectTypeException;
import java.lang.invoke.VarHandle;
import java.util.Map;

public class Vision
{
    public enum ETagType
    {
        None,
        Source,
        Speaker,
        SpeakerSide,
        Amp,
        Stage;
        
        ETagType() { this(0); }

        ETagType(double distanceOffset)
        {
            DistanceOffset = distanceOffset;
        }
        
        public final double DistanceOffset;
    }
    
    static private final Map<Integer, ETagType> TagInfo = Map.ofEntries
        (
              // Blue Source
              Map.entry(1, ETagType.Source)
            , Map.entry(2, ETagType.Source)
            
              // Red Speaker 
            , Map.entry(3, ETagType.SpeakerSide)
            , Map.entry(4, ETagType.Speaker)
              
              // Red Amp
            , Map.entry(5, ETagType.Amp)
              
              // Blue Amp
            , Map.entry(6, ETagType.Amp)
              
              // Blue Speaker
            , Map.entry(7, ETagType.Speaker)
            , Map.entry(8, ETagType.SpeakerSide)
              
              // Red Source
            , Map.entry(9, ETagType.Source)
            , Map.entry(10, ETagType.Source)

              // Red Stage
            , Map.entry(11, ETagType.Stage)
            , Map.entry(12, ETagType.Stage)
            , Map.entry(13, ETagType.Stage)

              // Red Stage
            , Map.entry(14, ETagType.Stage)
            , Map.entry(15, ETagType.Stage)
            , Map.entry(16, ETagType.Stage)
        );

    static private boolean ResultsAreStale = true;
    static private LimelightHelpers.Results LatestResults = null;


    static public void Periodic()
    {
        ResultsAreStale = true;
    }


    static private void UpdateResults()
    {
        
        if (ResultsAreStale)
        {
            LatestResults = LimelightHelpers.getLatestResults("").targetingResults;
            ResultsAreStale = false;
        }
    }
    
    static public ETagType GetTagTypeForTarget(LimelightHelpers.LimelightTarget_Fiducial target)
    {
        if (target == null) { return ETagType.None; }
        return TagInfo.getOrDefault((int)target.fiducialID, ETagType.None);
    }

    static public LimelightHelpers.LimelightTarget_Fiducial GetBestTarget()
    {
        UpdateResults();

        LimelightHelpers.LimelightTarget_Fiducial bestTarget = null;

        // -- loop through all fiducial targets and find the valid target that's closest to the center
        for (LimelightHelpers.LimelightTarget_Fiducial target : LatestResults.targets_Fiducials)
        {
            ETagType targetType = GetTagTypeForTarget(target);
            if (targetType != ETagType.None && targetType != ETagType.SpeakerSide)
            {
                if (bestTarget == null || Math.abs(target.tx) < Math.abs(bestTarget.tx))
                {
                    bestTarget = target;
                }
            }
        }

        return bestTarget;
    }

    static public LimelightHelpers.LimelightTarget_Detector GetBestNoteTarget()
    {
        LimelightHelpers.LimelightTarget_Detector bestNoteTarget = null;

        for (LimelightHelpers.LimelightTarget_Detector target : LatestResults.targets_Detector)
        {
            if (bestNoteTarget == null || Math.abs(target.tx) < Math.abs(bestNoteTarget.tx))
            {
                bestNoteTarget = target;
            }
        }
        return bestNoteTarget;
    }
}
