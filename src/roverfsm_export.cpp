// roverfsm_export.cpp
#include "roverfsmwithEnums.h"

extern "C" {
    __declspec(dllexport) with_enums::FSM* createFSM() { return new with_enums::FSM(); }
    __declspec(dllexport) int getState(with_enums::FSM* fsm) { return static_cast<int>(fsm->getState()); }
    __declspec(dllexport) void process(with_enums::FSM* fsm) { fsm->process(); }
    __declspec(dllexport) void reset(with_enums::FSM* fsm) { fsm->reset(); }
    __declspec(dllexport) void destroyFSM(with_enums::FSM* fsm) { delete fsm; }
}