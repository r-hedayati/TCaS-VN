#include "Probability.h"
#include "veins/modules/messages/WaveShortMessage_m.h"
#include <iostream>
#include <cstdio>
#include <cstring>
using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using std::sprintf;
Define_Module(Probability)
void Probability::initialize(int stage)
{
BaseWaveApplLayer::initialize(stage);
if (stage == 0) {// std::cerr << "In Probability::initialize()" << endl;
ProbabilityThreshold = par("ProbabilityThreshold").doubleValue();
indexOfAccidentNode = par("indexOfAccidentNode").longValue();
randomRebroadcastDelay = par("randomRebroadcastDelay").doubleValue();
traci = TraCIMobilityAccess().get(getParentModule());
stats = FranciscoStatisticsAccess().getIfExists();
ASSERT(stats);
beaconReceivedSignal = registerSignal("beaconReceivedSignal");
warningReceivedSignal = registerSignal("warningReceivedSignal");
messageReceivedSignal = registerSignal("messageReceivedSignal");
newWarningReceivedSignal = registerSignal("newWarningReceivedSignal");
lastDroveAt = simTime();
sentMessage = false;
}
}
void Probability::receiveSignal(cComponent *source, simsignal_t signalID, cComponent::cObject
*obj)
{
Enter_Method_Silent();
if (signalID == mobilityStateChangedSignal) {
handlePositionUpdate(obj);
}
}
void Probability::onBeacon(WaveShortMessage *wsm)
{
// std::cerr << "In Probability::onBeacon()" << endl;
}
void Probability::onData(WaveShortMessage *wsm)
{
// std::cerr << "In Probability::onData()" << std::endl;
emit(warningReceivedSignal, 1);
emit(messageReceivedSignal, 1);
stats->updateAllWarningsReceived();
stats->updateAllMessagesReceived();
// prevent originating disseminator from participating in further dissemination attempts
if (sentMessage)
return;
receivedMessages[wsm->getTreeId()].push_back(wsm->dup());
// is it a new warning message?
if (receivedMessages[wsm->getTreeId()].size() == 1) {
stats->updateNewWarningsReceived();
emit(newWarningReceivedSignal, 1);
char buf[64];
sprintf(buf, "%ld", wsm->getTreeId());
scheduleAt(simTime() + SimTime(randomRebroadcastDelay, SIMTIME_MS), new
cMessage(buf));
}
}
void Probability::handlePositionUpdate(cComponent::cObject *obj)
{
// stopped for for at least 10s?
if (traci->getSpeed() < 1) {
if ((simTime() - lastDroveAt >= 10)
&& (!sentMessage)
&& (indexOfAccidentNode == getParentModule()->getIndex())) {
std::cerr << "[DEBUG] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: "
<< getParentModule()->getIndex() << endl;
findHost()->getDisplayString().updateWith("r=16,red");
if (!sentMessage)
sendMessage(traci->getRoadId());
}
}
else {
lastDroveAt = simTime();
}
}
void Probability::sendMessage(std::string blockedRoadId)
{
t_channel channel = dataOnSch ? type_SCH : type_CCH;
WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
wsm->setWsmData(blockedRoadId.c_str());
sendWSM(wsm);
sentMessage = true;
}
void Probability::handleSelfMsg(cMessage *msg)
{
if ((!strcmp(msg->getName(), "data")) || (!strcmp(msg->getName(), "beacon"))) {
BaseWaveApplLayer::handleSelfMsg(msg);
return;
}
else { // IS A REBROADCAST
if (uniform(0,1)< ProbabilityThreshold)
return;
sendWSM(receivedMessages[atol(msg->getName())][0]->dup());
}
}
