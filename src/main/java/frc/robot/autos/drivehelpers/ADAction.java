package frc.robot.autos.drivehelpers;

import java.util.function.Function;

import frc.robot.autos.drivehelpers.helpers.ShootAimHelper;
import frc.robot.autos.drivehelpers.helpers.ShootRangeHelper;
import frc.robot.autos.drivehelpers.helpers.ShootStrafeHelper;
import frc.robot.ui.UI_Mode;

public enum ADAction {

    NONE(
        null,
        ADActionType.FULL,
        false,
        null,
        null
    ),
        
    SHOOT_AUTO_AIM(
        UI_Mode.SHOOT,
        ADActionType.FULL,
        true,
        null,
        m_ctx -> new ShootAimHelper(m_ctx, AdMetadataLibrary.SHOOT_AIM)
    ),

    SHOOT_AUTO_RANGE(
        UI_Mode.SHOOT,
        ADActionType.FULL,
        true,
        null,
        m_ctx -> new ShootRangeHelper(m_ctx, AdMetadataLibrary.SHOOT_RANGE)
    ),

    SHOOT_AUTO_STRAFE(
        UI_Mode.SHOOT,
        ADActionType.FULL,
        true,
        null,
        m_ctx -> new ShootStrafeHelper(m_ctx, AdMetadataLibrary.SHOOT_STRAFE)
    );

    public final UI_Mode m_requiredMode;
    public final ADActionType m_adType;
    public final boolean m_isAutoStart;
    public final ADHelperMetadata m_adMetadata;
    public final Function<ADContext, ContinuousAction> m_helperFactory;

    ADAction(UI_Mode mode, 
             ADActionType adType,
             boolean isAutoStart,
             ADHelperMetadata metadata, 
             Function<ADContext, ContinuousAction> helperFactory) {
        this.m_requiredMode = mode;
        this.m_adType = adType;
        this.m_isAutoStart = isAutoStart;
        this.m_adMetadata = metadata;
        this.m_helperFactory = helperFactory;
    }

    public enum ADActionType { ROTATION, TRANSLATION, FULL }
}