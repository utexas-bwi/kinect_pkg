#include <kinect_pkg/KFRArray.h>


KFRArray::KFRArray() {}
KFRArray::~KFRArray() {}

void KFRArray::receiveFrame(KinectFrame *frame) {
    for(int i = 0; i < _kfr.size(); i++) {
        _kfr[i]->receiveFrame(frame);
    }
}

void KFRArray::addKFR(KinectFrameRecipient *kfr) {
    _kfr.push_back(kfr);
}