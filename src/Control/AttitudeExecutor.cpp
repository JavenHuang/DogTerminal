#include "AttitudeExecutor.h"
#include "DogLink/ZsibotDefs.h"

#include <QDebug>

AttitudeExecutor::AttitudeExecutor(QObject *parent)
    : QObject(parent)
{
}

void AttitudeExecutor::execute(DogProtocol::AttitudeAction action)
{
    using namespace DogProtocol;
    quint32 code = Zsibot::CMD_NULL;

    switch (action)
    {
    case AA_STAND_UP:        code = Zsibot::CMD_STAND_UP;          break;
    case AA_SIT_DOWN:        code = Zsibot::CMD_SIT_DOWN;          break;
    case AA_BALANCE_STAND:   code = Zsibot::CMD_BALANCE_STAND_MODE; break;
    case AA_LOCK:            code = Zsibot::CMD_LOCK_MODE;         break;
    case AA_EMERGENCY_STOP:  code = Zsibot::CMD_EMERGENCY_STOP;    break;
    case AA_MOVE_MODE:       code = Zsibot::CMD_MOVE_MODE;         break;
    case AA_SLOW_SPEED:      code = Zsibot::CMD_SLOW_SPEED;        break;
    case AA_NORMAL_SPEED:    code = Zsibot::CMD_NORMAL_SPEED;      break;
    case AA_FAST_SPEED:      code = Zsibot::CMD_FAST_SPEED;        break;
    case AA_REQUEST_CONTROL: code = Zsibot::CMD_SDK_CONTROL_RIGHT; break;
    default:
        emit unknownAction(action);
        return;
    }

    emit cmdCode(code);
}
