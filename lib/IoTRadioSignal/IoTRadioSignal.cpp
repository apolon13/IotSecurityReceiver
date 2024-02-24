#include <sstream>
#include "IoTRadioSignal.h"

using namespace std;

IoTRadioSignal::IoTRadioSignal(int rp) : receivePin(rp) {
    rcs = new RCSwitch();
}

void IoTRadioSignal::scan(string &value) {
    if (rcs->available()) {
        string rcv = to_string(rcs->getReceivedValue());
        reset();
        value = rcv;
    }
}

void IoTRadioSignal::reset() {
    rcs->resetAvailable();
}

void IoTRadioSignal::enable() {
    rcs->enableReceive(receivePin);
}

void IoTRadioSignal::disable() {
    rcs->disableReceive();
}

