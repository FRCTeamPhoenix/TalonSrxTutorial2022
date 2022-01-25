#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <ctre/Phoenix.h>

#include "Constants.h"

class DriveSubsystem : public frc2::SubsystemBase {
    public:
        DriveSubsystem();

        void Periodic();

        void Tankdrive(double left, double right);

        void ConfigureDefault();

        void ZeroEncoders();

        bool IsStopped();

        //average of both wheels in terms of error
        units::meter_t ClosedLoopError();

        //called periodically
        void RunMotionMagic(units::meter_t distance);

        //called periodically
        void RunPID(units::meter_t distance);
    private:
        units::meter_t TicksToDistance(double ticks);
        double DistanceToTicks(units::meter_t distance);

        TalonSRX m_frontLeft{TALON_FRONT_LEFT};
        TalonSRX m_frontRight{TALON_FRONT_RIGHT};
        VictorSPX m_backLeft {VICTOR_BACK_LEFT};
        VictorSPX m_backRight {VICTOR_BACK_RIGHT};
};