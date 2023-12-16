package frc.team5406.robot.lib.navx;

interface IBoardCapabilities {
    public boolean isOmniMountSupported();
    public boolean isBoardYawResetSupported();
    public boolean isDisplacementSupported();
    public boolean isAHRSPosTimestampSupported();
}
