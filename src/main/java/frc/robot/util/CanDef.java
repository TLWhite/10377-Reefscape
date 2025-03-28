package frc.robot.util;
// This was brought in from 930 code in an attept to use it.  brought in Gains.java, loggedTuneablenumber.java,
// LoggedTunableGainsBuilder.java, and LoggerTracker.java..  TLWhite
public class CanDef {
    private final int id;
    private final CanBus bus;

    public enum CanBus {
        Rio("rio"),
        CANivore("ARM");

        private final String name;

        private CanBus(String name) {
            this.name = name;
        }
    }

    private CanDef(int id, CanBus bus) {
        if (id < 0) {
            throw new IllegalArgumentException("CAN ID (" + id + ", " + bus.name + ") must be non-negative");
        }

        this.id = id;
        this.bus = bus;
    }

    public int id() {
        return id;
    }

    public String bus() {
        return bus.name;
    }

    public static Builder builder() {
        return new Builder();
    }

    // create a builder for this class
    public static class Builder {
        private int id = -1;
        private CanBus bus = CanBus.Rio;

        public Builder id(int id) {
            this.id = id;
            return this;
        }

        public Builder bus(CanBus bus) {
            this.bus = bus;
            return this;
        }

        public CanDef build() {
            return new CanDef(id, bus);
        }
    }
}
