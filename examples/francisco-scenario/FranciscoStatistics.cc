#include "FranciscoStatistics.h"
#include <iostream>
using std::cerr;
using std::endl;
Define_Module(FranciscoStatistics)
void FranciscoStatistics::initialize(int stage)
{
if (stage == 0) {
allBeaconsReceivedSignal = registerSignal("allBeaconsReceivedSignal");
allNewWarningsReceivedSignal = registerSignal("allNewWarningsReceivedSignal");
allWarningsReceivedSignal = registerSignal("allWarningsReceivedSignal");
allMessagesReceivedSignal = registerSignal("allMessagesReceivedSignal");
allBeaconsReceived = allWarningsReceived = newWarningsReceived = allMessagesReceived =
0;
numAccidentsOccurred = 0;
}
}
void FranciscoStatistics::finish()
{ }
void FranciscoStatistics::updateAllBeaconsReceived()
{
++allBeaconsReceived;
emit(allBeaconsReceivedSignal, allBeaconsReceived);
}
void FranciscoStatistics::updateNewWarningsReceived()
{
++newWarningsReceived;
emit(allNewWarningsReceivedSignal, newWarningsReceived);
// cerr << "num warnings: " << newWarningsReceived << simTime().str() << endl;
}
void FranciscoStatistics::updateAllWarningsReceived()
{
emit(allWarningsReceivedSignal, ++allWarningsReceived);
}
void FranciscoStatistics::updateAllMessagesReceived()
{
emit(allMessagesReceivedSignal, ++allMessagesReceived);
}
void FranciscoStatistics::incrementAccidentOccurred()
{
emit(numAccidentsSignal, ++numAccidentsOccurred);
}