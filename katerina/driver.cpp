#include "driver.h"

const float Driver::MAX_UNSTUCK_ANGLE = 30.0/180.0*PI; // [radians]
const float Driver::UNSTUCK_TIME_LIMIT = 2.0;

Driver::Driver(int index)
{
    INDEX = index;
}

// Called for every track change or new race
void Driver::initTrack(tTrack* t, void *carHandle,
                       void **carParmHandle, tSituation *s)
{
    track = t;
    *carParmHandle = NULL;
}

// Start a new race
void Driver::newRace(tCarElt* car, tSituation *s)
{
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
    stuck = 0;
}

// Drive during race
void Driver::drive(tCarElt* car, tSituation *s)
{
    update(car, s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck(car)) {
        car->ctrl.steer = -angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.3;
        car->ctrl.brakeCmd = 0.0;
    } else {
        float steerangle = angle - car->_trkPos.toMiddle/car->_trkPos.seg->width;
        car->ctrl.steer = steerangle / car->_steerLock;
        car->ctrl.gear = 1; // first gear
        car->ctrl.accelCmd = 0.3;
        car->ctrl.brakeCmd = 0.0;
    }
}

// Set pitstop commands
int Driver::pitCommand(tCarElt* car, tSituation *s)
{
    return ROB_PIT_IM; // return immediately
}

// End of the current race
void Driver::endRace(tCarElt *car, tSituation *s)
{
}

// Update private data every timestep
void Driver::update(tCarElt* car, tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
}

// Check if stuck
bool Driver::isStuck(tCarElt* car)
{
    if (fabs(angle) < MAX_UNSTUCK_ANGLE) {
        stuck = 0;
        return false;
    }
    if (stuck < MAX_UNSTUCK_COUNT) {
        stuck++;
        return false;
    } else {
        return true;
    }
}
