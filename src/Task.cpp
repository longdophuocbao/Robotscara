#include "Task.h"

// ON - turn on the Knift
void SetKnift(String _Status)
{
    if (_Status == "ON")
        digitalWrite(Knift, HIGH);
    else
        digitalWrite(Knift, LOW);
}
