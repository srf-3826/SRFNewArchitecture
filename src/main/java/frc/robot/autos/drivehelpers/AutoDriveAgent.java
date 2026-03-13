package frc.robot.autos.drivehelpers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.TargetInfo;

public class AutoDriveAgent {

    public class OwnedHelper {
        public final ADAction m_owner;
        public final ContinuousAction m_helper;
        public Optional<TargetInfo> m_selectedTag = Optional.empty();

        public OwnedHelper( ADAction owner, 
                            ContinuousAction helper, 
                            Optional<TargetInfo> selectedTag ) {
            this.m_owner = owner;
            this.m_helper = helper;
            this.m_selectedTag = selectedTag;
        }
    }

    private final List<OwnedHelper> m_activeHelpers = new ArrayList<>();
    private final ADContext m_adCtx;

    private boolean m_shootDriveHelpersEnabled = false;
    private boolean m_autoDriveHelpersOK = false;

    // construct a new AutoDriveAgent
    public AutoDriveAgent(ADContext adCtx) {
        this.m_adCtx = adCtx;
    }

    // The following two methods enable and disable AutoDriveHelpersOK.
    // Set manually via a press / Alt press (typ. on Button B)
    // The actual Helpers to be used will depend on the UI_Mode. 
    public void enableAutoDriveHelpers() { m_autoDriveHelpersOK = true; }
    public void disableAutoDriveHelpers() { m_autoDriveHelpersOK = false; }
	
    public boolean isShootAutoDriveHelpersEnabled() { return m_shootDriveHelpersEnabled; }

    // SelectTagFor returns an optional TargetInfo whose Target Tag ID matches 
    // at least one of the ReuiredTagIDs specified in ADHelperMetadata. It also
    // must satisfy the tagSelectionMode, also specified in the metadata
    public Optional<TargetInfo> selectTagFor(ADHelperMetadata adMetadata) {

        List<TargetInfo> all = m_adCtx.vision.getAllTargetInfo();
        if (all.isEmpty()) return Optional.empty();

        // Filter by required tag IDs
        List<TargetInfo> valid = all.stream().filter(
                                              t -> Arrays.stream(adMetadata.requiredTagIds)
                                              .anyMatch(id -> id == t.tagId)
                                              ).toList();

        if (valid.isEmpty()) return Optional.empty();

        // Select based on metadata
        return switch (adMetadata.tagSelectionMode) {
            case NEAREST ->
                valid.stream().min(Comparator.comparingDouble(TargetInfo::distanceErrorMeters));
            case FARTHEST ->
                valid.stream().max(Comparator.comparingDouble(TargetInfo::distanceErrorMeters));
            case FIRST_VALID ->
                Optional.of(valid.get(0));
        };
    }
    
    public void update() {

        // 1. Remove finished helpers
        m_activeHelpers.removeIf(h -> {
            if (h.m_helper.isFinished()) {
                h.m_helper.stop();
                return true;
            }
            return false;
        });

        // 2. Loop over all ADAction values
        for (ADAction action : ADAction.values()) {

            if (!action.m_isAutoStart) continue;
            if (action == ADAction.NONE) continue;
            if (action.m_requiredMode != m_adCtx.modeManager.getMode()) continue;

            ContinuousAction helper = action.m_helperFactory.apply(m_adCtx);

            Optional<TargetInfo> tag = selectTagFor(action.m_adMetadata);
            if (!helper.shouldActivate(m_adCtx, tag, m_autoDriveHelpersOK)) continue;
            if (hasHelperOfType(helper.getClass())) continue;

            helper.start();
            m_activeHelpers.add(new OwnedHelper(action, helper, selectTagFor(action.m_adMetadata)));
        }

        // 3. Update all active helpers
        for (OwnedHelper oh : m_activeHelpers) {
            oh.m_selectedTag = selectTagFor(oh.m_owner.m_adMetadata);
            oh.m_helper.update(oh.m_selectedTag, m_shootDriveHelpersEnabled);
        }
    }

    // getAutoDriveAssistSpeeds returns the sum of all ChassisSpeeds calculated
    // by any active helpers. It is typically called only by DefaultDriveCmd.
    public ChassisSpeeds getAutoDriveAssistSpeeds() {
        double vx = 0;
        double vy = 0;
        double omega = 0;

        for (OwnedHelper oh : m_activeHelpers) {
            ChassisSpeeds s = oh.m_helper.getSpeeds();
            vx += s.vxMetersPerSecond;
            vy += s.vyMetersPerSecond;
            omega += s.omegaRadiansPerSecond;
        }

        return new ChassisSpeeds(vx, vy, omega);
    }

    // hasHelperOfType returns true if and only if the list of activeHelpers has a member that
    // matches the class clazz. passed in as the sole argument.
    private boolean hasHelperOfType(Class<?> clazz) {
        return m_activeHelpers.stream().anyMatch(h -> h.m_helper.getClass() == clazz);
    }
}