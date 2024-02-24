#include "RCSwitch.h"
#include <vector>

using namespace std;

#ifndef DISPLAY_SENSOR_H
#define DISPLAY_SENSOR_H


class IoTRadioSignal {
protected:
    RCSwitch *rcs;
    int receivePin;
public:
    explicit IoTRadioSignal(int receivePin);
    void scan(string &value);
    void reset();
    void enable();
    void disable();
};
#endif //DISPLAY_SENSOR_H
