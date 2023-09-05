package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    
    protected final NetworkTable table;
    protected NetworkTableEntry tx;
    protected NetworkTableEntry ty;
    protected NetworkTableEntry ta;


    public Limelight() {
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.tx = getNewTxEntry();
        this.ty = getNewTyEntry();
        this.ta = getNewTaEntry();
    }

    /**
     * update the tx, ty, and ta values from the limelight
     */
    public void update() {
        this.updateTx();
        this.updateTy();
        this.updateTa();
    }

    /**
     * @set the protected tx, ty, and ta to the new values from the limelight
     */
    public void updateTx() {
        this.tx = getNewTxEntry();
    }
    public void updateTy() {
        this.ty = getNewTyEntry();
    }
    public void updateTa() {
        this.ta = getNewTaEntry();
    }

    /**
     * @return the tx, ty, and ta values from the limelight
     */
    public NetworkTableEntry getNewTxEntry() {
        return this.table.getEntry("tx");
    }
    public NetworkTableEntry getNewTyEntry() {
        return this.table.getEntry("ty");
    }
    public NetworkTableEntry getNewTaEntry() {
        return this.table.getEntry("ta");
    }

    /**
     * 
     * @return the tx, ty, and ta values from the protected variables
     */
    public NetworkTableEntry[] getEntries() {
        return new NetworkTableEntry[] {this.tx, this.ty, this.ta};
    }
    public double[] getValues() {
        return new double[] {this.getTxValue(), this.getTyValue(), this.getTaValue()};
    }
    
    public double getTxValue() {
        return this.tx.getDouble(0.0);
    }
    public double getTyValue() {
        return this.ty.getDouble(0.0);
    }
    public double getTaValue() {
        return this.ta.getDouble(0.0);
    }
}
