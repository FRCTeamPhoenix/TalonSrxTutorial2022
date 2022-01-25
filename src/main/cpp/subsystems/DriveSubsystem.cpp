#include "subsystems/DriveSubsystem.h"

#include <wpi/math>

DriveSubsystem::DriveSubsystem(){
    ConfigureDefault();
}

void DriveSubsystem::Periodic(){

}

void DriveSubsystem::Tankdrive(double left, double right){
    m_frontLeft.Set(ControlMode::PercentOutput, left);
    m_frontRight.Set(ControlMode::PercentOutput, right);
}

void DriveSubsystem::ConfigureDefault(){
    //reset everything
    m_frontLeft.ConfigFactoryDefault();
    m_frontRight.ConfigFactoryDefault();
    m_backLeft.ConfigFactoryDefault();
    m_backRight.ConfigFactoryDefault();

    //back motors follow front motors
    m_backLeft.Follow(m_frontLeft);
    m_backRight.Follow(m_frontRight);

    //make sure sensors match forward direction
    m_frontRight.SetSensorPhase(false);
    m_frontLeft.SetSensorPhase(false);

    //make the motors go in the arbitrary forward direction
    m_frontLeft.SetInverted(false);
    m_backLeft.SetInverted(false);
    m_frontRight.SetInverted(false);
    m_backRight.SetInverted(false);

    //set the sensor type to a relative mag encoder
    m_frontLeft.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
    m_frontRight.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);

    //10 ms period
    m_frontLeft.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
    m_frontRight.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
    m_frontLeft.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    m_frontRight.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);

    //nominal = default, peak = maximum
    m_frontLeft.ConfigNominalOutputForward(0.0, 10);
    m_frontLeft.ConfigNominalOutputReverse(0.0, 10);
    m_frontLeft.ConfigPeakOutputForward(1.0, 10);
    m_frontLeft.ConfigPeakOutputReverse(-1.0, 10);
    m_frontRight.ConfigNominalOutputForward(0.0, 10);
    m_frontRight.ConfigNominalOutputReverse(0.0, 10);
    m_frontRight.ConfigPeakOutputForward(1.0, 10);
    m_frontRight.ConfigPeakOutputReverse(-1.0, 10);

    //configure the distance PID values
    m_frontLeft.SelectProfileSlot(0, 0);
    m_frontLeft.Config_kP(0, DRIVETRAIN_DISTANCE_P);
    m_frontLeft.Config_kI(0, DRIVETRAIN_DISTANCE_I);
    m_frontLeft.Config_kD(0, DRIVETRAIN_DISTANCE_D);
    m_frontLeft.Config_kF(0, DRIVETRAIN_DISTANCE_F);
    m_frontRight.SelectProfileSlot(0, 0);
    m_frontRight.Config_kP(0, DRIVETRAIN_DISTANCE_P);
    m_frontRight.Config_kI(0, DRIVETRAIN_DISTANCE_I);
    m_frontRight.Config_kD(0, DRIVETRAIN_DISTANCE_D);
    m_frontRight.Config_kF(0, DRIVETRAIN_DISTANCE_F);

    //TODO: Add motion magic stuff

    ZeroEncoders();
}

void DriveSubsystem::ZeroEncoders(){
    m_frontLeft.SetSelectedSensorPosition(0, 0, 10);
    m_frontRight.SetSelectedSensorPosition(0, 0, 10);
}

bool DriveSubsystem::IsStopped(){
    //TODO: Configure velocity detection stuff
}

//average of both wheels in terms of error
units::meter_t DriveSubsystem::ClosedLoopError(){
    double average = (m_frontLeft.GetClosedLoopError(0) + m_frontRight.GetClosedLoopError(0)) / 2.0;
    return TicksToDistance(average);
}

//called periodically
void DriveSubsystem::RunMotionMagic(units::meter_t distance){
    double ticksDistance = DistanceToTicks(distance);
    m_frontLeft.Set(ControlMode::MotionMagic, ticksDistance);
    m_frontRight.Set(ControlMode::MotionMagic, ticksDistance);
}

void DriveSubsystem::RunPID(units::meter_t distance){
    double ticksDistance = DistanceToTicks(distance);
    m_frontLeft.Set(ControlMode::Position, ticksDistance);
    m_frontRight.Set(ControlMode::Position, ticksDistance);
}

units::meter_t DriveSubsystem::TicksToDistance(double ticks){
    return ticks / TICKS_PER_ROTATION * wpi::math::pi * WHEEL_DIAMETER;
}

double DriveSubsystem::DistanceToTicks(units::meter_t distance){
    return distance / wpi::math::pi / WHEEL_DIAMETER * TICKS_PER_ROTATION;
}