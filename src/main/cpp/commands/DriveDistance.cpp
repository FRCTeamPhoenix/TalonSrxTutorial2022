#include "commands/DriveDistance.h"

DriveDistance::DriveDistance(DriveSubsystem* driveSubsystem, units::meter_t distance):
m_driveSubsystem(driveSubsystem),
m_distance(distance){
    AddRequirements(driveSubsystem);
}

void DriveDistance::Initialize(){
    m_driveSubsystem->ZeroEncoders();
    m_driveSubsystem->Tankdrive(0.0, 0.0);
}

void DriveDistance::Execute(){
    m_driveSubsystem->RunMotionMagic(m_distance);
}

void DriveDistance::End(bool interrupted){
    m_driveSubsystem->Tankdrive(0.0, 0.0);
}

bool DriveDistance::IsFinished(){
    return false;
}