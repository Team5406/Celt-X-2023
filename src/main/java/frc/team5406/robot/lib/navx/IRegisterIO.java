package frc.team5406.robot.lib.navx;

interface IRegisterIO {
    boolean init();
    boolean write(byte address, byte value );
    boolean read(byte first_address, byte[] buffer);
    boolean shutdown();
    void enableLogging(boolean enable);
}
