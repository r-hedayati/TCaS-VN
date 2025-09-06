/*
 * SERVitESApplLayerOLD.cc
 *
 *  Created on: Apr 3, 2018
 *      Author: Montajab Ghanem
 */


#include "SERVitESApplLayerOLD.h"
#include"string.h"
#include <stdio.h>
#include <stdio.h>
#include <string.h>

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t SERVitESApplLayerOLD::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(SERVitESApplLayerOLD);

void SERVitESApplLayerOLD::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        mobility = TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);
        sentMessage = false;
        lastDroveAt = simTime();
        findHost()->subscribe(parkingStateChangedSignal, this);
        isParking = false;
        sendWhileParking = par("sendWhileParking").boolValue();
        maxSpeed = par("maxSpeed");
        res = uniform(1,3);
        state = ND;//line 1
        counterSerial = 0;
        for (int i = 0; i< 1000; i++){
            stateSerial[i] = -1;
            serviceSerial[i] = -1;
        }
        stateSerial[counterSerial] = state;
        serviceDuration = 0.5;
        serviceDurationTime = 0 ;
        serviceSearchingTime=0;
        serviceSearchingTimeMIN=1;
        serviceSearchingTimeMAX=0;
        ID_Controller = -1;
        sourceID = -1 ;
        helpID = -1;
        csim = 0;
        ttl = 0;
        hopthreshold = 1;
        cellnumber = getCellNumber(curPosition);
        transmissionTime = 0.005;
        beaconInterval = 0.5;
        serviceInterval = uniform(5,10);
        serviceSearchingTime=0;
        kindGateway = 1;// All nodes can be an internal GW
        ID_requester = -1;
        requestNumber =0;
        requestHelpNumber=0;
        overheadCluster = 0;
        overheadService = 0;
        overheadBeacon = 0;

        receiveMsgDuringWaiting = false;
        waitingTimeExpireed = false;
        helloMsgSent = false;
        candidateMsgSent = false;
        controllerMsgSent = false;
        beaconMsgSent = false;
        sendQueryMessage = false;
        resAccess = true ;
        joinToCluster = false;

        CHDuration.setName("CH Duration");
        ServiceAvailability.setName("Service Availability");

        receivedControlMsgAfterCandidate = false;
        helloMsgTimer = new cMessage("HelloTimer",HELLO_MSG_TIMER);
        helloMsgTimer->setKind(HELLO_MSG_TIMER);
        waitingResponseTimer = new cMessage("waitingResponseTimer",WAITING_RESPONSE_TIMER);
        resetTimer = new cMessage("resetTimer",RESET_TIMER);
        scheduleAt(simTime() + 90, resetTimer);
        waitingResponseTimer->setKind(WAITING_RESPONSE_TIMER);
        scheduleAt(simTime() + 40, helloMsgTimer);
    }

}

void SERVitESApplLayerOLD::onBeacon(WaveShortMessage* wsm) {
}

void SERVitESApplLayerOLD::onData(WaveShortMessage* wsm) {
   findHost()->getDisplayString().updateWith("r=16,green");
   switch (wsm->getKind()){
        case HELLO_MSG :{// line 3
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
                cCarList::iterator it;
                bool carExist = false;
                for (it=carList.begin();it!=carList.end();++it){
                    if((wsm->getID()==it->id)){
                        carExist = true;
                    }
                }
                if ((carExist == false)&&(wsm->getID() != myId)){
                    carList.push_back(Car(wsm->getID()));
                    if (tsimCalculate(wsm->getWsmData(),mobility->getRoadId())>0){
                        csim++;
                    }
                    EV<<"Transmission time is updated : "<<transmissionTime<<endl;
                    EV<<"Hello message has been received from "<<wsm->getID()<<endl;
                    EV<<"Path for source is "<<wsm->getWsmData()<<endl;
                    EV<<"Path   for  me  is "<<mobility->getRoadId()<<endl;
                    EV<<"I have hello Msgs from: "<<endl;
                    for (it=carList.begin();it!=carList.end();++it){
                        EV<<"Node "<<it->id<<endl;
                    }
                    transmissionTime = simTime() - wsm->getTimestamp();

                    tsim = tsimCalculate(wsm->getWsmData(),mobility->getRoadId());
                    tcell = calculateTCell();
                    EV<<"tsim is "<<tsim<<", tcell is "<<tcell<<endl;
    //                double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
    //                tcell = d/(mobility->getSpeed()+1);
                    WaveShortMessage* ResponseMsg = new WaveShortMessage("SERVitES",RESPONSE_MSG);
                    t_channel channel = dataOnSch ? type_SCH : type_CCH;
                    ResponseMsg->addBitLength(headerLength);
                    ResponseMsg->addBitLength(dataLengthBits);
                    switch (channel) {
                       case type_SCH: ResponseMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
                       case type_CCH: ResponseMsg->setChannelNumber(Channels::CCH); break;
                    }
                    for (int r = 0; r < 10; r++){
                        ResponseMsg->setRoute(r,-1);
                    }
                    ResponseMsg->setRoute(0,myId);
                    //EV<<"response route is "<<ResponseMsg->getRoute(0)<<endl;
                    ResponseMsg->setPsid(0);
                    ResponseMsg->setPriority(dataPriority);
                    ResponseMsg->setWsmVersion(1);
                    ResponseMsg->setTimestamp(simTime());
                    ResponseMsg->setSenderAddress(myId);
                    ResponseMsg->setRecipientAddress(-1);
                    ResponseMsg->setSenderPos(curPosition);
                    ResponseMsg->setCellNumber(getCellNumber(curPosition));
                    ResponseMsg->setDestinationID(wsm->getSenderAddress());
                    ResponseMsg->setSerial(2);
                    ResponseMsg->setID(myId);//ID
                    ResponseMsg->setTtl(ttl);
                    ResponseMsg->setAngle(mobility->getAngleRad());
                    ResponseMsg->setWsmData(mobility->getRoadId().c_str());
                    ResponseMsg->setState(state);//state
                    ResponseMsg->setID_Controller(ID_Controller);//ID_Controller
                    ResponseMsg->setCsim(csim);//csim
                    if (tsim > 0){//tsim
                        ResponseMsg->setTsim(tsim.dbl());
                    }
                    else{
                        ResponseMsg->setTsim(0);
                    }
                    ResponseMsg->setTcell(tcell.dbl());//tcell
                    ResponseMsg->setKind(RESPONSE_MSG);
                    sendWSM(ResponseMsg);// line 4

                    overheadCluster++;
                    EV<<"Response message with state '"<<state<<"' is sent to "<<wsm->getID()<<endl;
                    if (wsm->getTtl() > 0){
                        relayPacket(wsm);
                    }
                }
            }
                break;
        }
        case RESPONSE_MSG : {//Condition line 7
//            EV<<"waitingTimeExpireed is "<<waitingTimeExpireed<<endl;
//            EV<<"helloMsgSent"<<helloMsgSent<<endl;
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
                if ((waitingTimeExpireed == false)&&(helloMsgSent == true)){// condition at line 6 of the Algorithm
                    receiveMsgDuringWaiting = true;
                    cCarList::iterator it;
                    bool carExist = false;
                    for (it=ResponsedCarList.begin();it!=ResponsedCarList.end();++it){
                        if(wsm->getID()==it->id){
                            carExist = true;
                        }
                    }
                    if (carExist == false){
                        ResponsedCarList.push_back(Car(wsm->getID()));
                        if ((tsimCalculate(wsm->getWsmData(),mobility->getRoadId())>0)&&(abs(mobility->getAngleRad()-wsm->getAngle()) <= 0.78)){
                            csim++;
                        }
                    }
                    EV<<"My csim is "<<csim;
                    EV<<". Received csim is "<<wsm->getCsim()<<endl;
                    EV<<"My tcell is "<<tcell;
                    EV<<". Received tcel is "<<wsm->getTcell()<<endl;
                    EV<<"I have received reponses Msgs from: "<<endl;
                    for (it=ResponsedCarList.begin();it!=ResponsedCarList.end();++it){
                        EV<<"Node "<<it->id<<endl;
                    }

                    if (wsm->getState() == ND){//Line 8
                        EV << "get state = ND "<<endl;
                        if ((csim > wsm->getCsim())&&(tcell > wsm->getTcell())){// Condition line 10
                            if (candidateMsgSent == false){
                                EV<<"Condition line 10 is satisfied!"<<endl;
                                sendCandidateMsg();// Line 11
                                overheadCluster++;
                                transmissionTime = simTime() - wsm->getTimestamp();
                                EV<<"Transmission time is updated(RESPONSE-->CANDIDATE) : "<<transmissionTime<<endl;
                                waitingAfterCandidateMsg = new cMessage("waitingAfterCandidateMsg",WAITING_AFTER_CANDIDATE_MSG);
                                scheduleAt(simTime()+5*(transmissionTime+individualOffset),waitingAfterCandidateMsg);
                            }
                        }
                        else {// Line 12
                            state = MV;// Line 13
                            EV<<"State is set to MV"<<endl;
                            ID_Controller = wsm->getID_Controller();
                            EV<<"Controller is "<<ID_Controller<<endl;
                            if (ID_Controller!=-1){
                                joinToCluster = 1;
                            }
                            // Line 14 Store the information
                        }// Line 15
                    }// Line 16
                    if ((wsm->getState() == CH)||(wsm->getState() == MV)){//Line 17
                        EV << "get state = CH "<<endl;
                        if (ID_Controller!=-1){
                            joinToCluster = 1;
                        }
                        maintenance();// Line 18
                    }// Line 19
                }
                if ((wsm->getDestinationID() != myId)&&(wsm->getTtl() > 0)){
                    relayPacket(wsm);
                }
            }
            break;
        }// Line 20
        case CANDIDATE_MSG:{ // Line 21
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
                EV<<"Candidate message received!"<<endl;
                // Analysis the information, Line 22
                if ((csim >= wsm->getCsim())&&(tcell >= wsm->getTcell())){   //Line 23
                    EV<<"Change state to CH!"<<endl;
                    state = CH; // Line 24
                    ID_Controller = myId;
                    sendControllerMsg(); // Line 25;
                    overheadCluster++;
                    joinToCluster =true;
                }
                else {// Line 26
                    state = MV ;
                    ID_Controller = wsm->getID_Controller();
                    EV<<"Controller is "<<ID_Controller<<endl;
                    if (ID_Controller!=-1){
                        joinToCluster = 1;
                    }
                    // Store the Information!
                }// Line 27
            }
            break;
        }// Line 28
        case CONTROL_MSG:{
            EV<<"Control message received!"<<endl;
            receivedControlMsgAfterCandidate = true;
            int numberOfController = 0;
            cControllerList::iterator it;
            bool controllerExist = false;// to check if the received controlMsg
                //had been received from existed controller
            for (it=controllerList.begin();it!=controllerList.end();++it){
                if(wsm->getID()==it->id){
                    controllerExist = true;
                }
            }
            if (controllerExist == false){
                controllerList.push_back(Controller(wsm->getID(),wsm->getTcell(),wsm->getTsim(),
                        wsm->getCsim(),ttl - wsm->getTtl()));
            }
            int hcount [5];
            for (it=controllerList.begin();it!=controllerList.end();++it){
                EV<<"Controller ID is "<<it->id<<endl;
                hcount[numberOfController] = it->hopCount;
                numberOfController++;
            }
            if (numberOfController == 1){//If there is only one CH in the vicinity, then a potential CH
                //will accept the vehicle in case their hop is lower than a predefined threshold
                if (hcount[0] < hopthreshold){
                    ID_Controller = wsm->getID_Controller();
                    state = MV;
                    EV<<"Change state to MV!"<<endl;
                    if (ID_Controller!=-1){
                        joinToCluster = 1;
                    }
                }
                else {//Otherwise, the vehicle becomes a new Controller and creates a cluster.
                    EV<<"Change state to CH!"<<endl;
                    state = CH;
                    ID_Controller = myId;
                    sendControllerMsg();
                    overheadCluster++;
                    joinToCluster =true;
                }
            }
            if (numberOfController > 1){//In the event that more than one
                //cluster head exists, the vehicle joins a cluster where
                //it will remain for the greatest trajectory similarity,
                controllerList.sort();
                it=controllerList.begin()++;
                ID_Controller = it->id;
                state = GW;
                //vehicles that have connections with other clusters.
                //These vehicles work as a communication bridge between
                //clusters. When a vehicle succeeds in communicating with other
                //clusters, it propagates a gateway message.
                kindGateway = 2;
                ID_Gateway = myId;
                if (ID_Controller!=-1){
                    joinToCluster = 1;
                }
                sendGatewayMsg();
                overheadCluster++;
            }
            //state = MV;//After selecting the cluster,
            //the vehicle stores the information, becomes an MV, and sends
            //a Join Message to the CH to direct the Controller to join a new MV.

            if ((res <= 2)&&(sendQueryMessage == false)){ // Algorithm 2, line 1
                //sendQueryMsg();// Algorithm 2, line 2
                //serviceSearchingTime = simTime();
                //sendQueryMessage = true;
                serviceIntervalTimer = new cMessage("serviceIntervalTimer",SERVICE_INTERVAL_TIMER);
                serviceIntervalTimer->setKind(SERVICE_INTERVAL_TIMER);
                if (serviceIntervalTimer->isScheduled() != true){//After creating the cluster,
                       //each vehicle inside a cluster periodically sends a service request message to the Controller,
                scheduleAt(simTime() + serviceInterval, serviceIntervalTimer);
                            }
            }
            ID_Controller = wsm->getID_Controller();
            sendJoinMsg();
            overheadCluster++;
            if (ID_Controller!=-1){
                joinToCluster = 1;
            }


            //sendBeaconMsg();
            beaconTimer = new cMessage("beaconTimer",BEACON_TIMER);
            beaconTimer->setKind(BEACON_TIMER);
            if (beaconTimer->isScheduled() != true){//After creating the cluster,
                    //each vehicle inside a cluster periodically sends a beacon message to the Controller,
                scheduleAt(simTime() + beaconInterval, beaconTimer);
            }
            break;
        }
        case GATEWAY_MSG: {
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
                if(state == MV){
                    EV<<"Gateway message received!"<<endl;
                    if (ID_Controller != -1){
                        if (ID_Controller == wsm->getID_Controller()){
                            EV<<"ID Gateway is set!"<<endl;
                            ID_Gateway = wsm->getID_Gateway();
                        }
                    }
                }
            }
        }
        case BEACON_MSG:{
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
            //if (ID_Controller == wsm->getID_Controller()){
                if ((state == GW)||(state == CH)){
                    EV<<"Beacon message received from "<< wsm->getID()<<endl;
                    EV << "my ID is : " << myId << "Controller ID is :" << ID_Controller << endl;
                    cConnectedVList::iterator it;
                    bool carExistGW = false;// to check if the received controlMsg
                            //had been received from existed controller
                    for (it=connectedVList.begin();it!=connectedVList.end();++it){
                        if(wsm->getID()==it->id){
                            carExistGW = true;
                            connectedVList.erase(it);
                        }
                    }
                    connectedVList.push_back(ConnectedV(wsm->getID(),wsm->getTcell(),wsm->getTsim(),
                            wsm->getCsim(),ttl - wsm->getTtl(),wsm->getRes()));
                }
            //}
            }
            break;
        }
        case JOIN_MSG:{
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
            //if (ID_Controller == wsm->getID_Controller()){
                cMemberVList::iterator it;
                if ((state == CH)&&(myId==wsm->getID_Controller())){
                    if((csim > wsm->getCsim())&&(tcell > wsm->getTcell())){
                        EV<<"Join message received from "<<wsm->getID()<<endl;
                        bool memberExist = false;
                        for (it=memberVList.begin();it!=memberVList.end();++it){
                            if(wsm->getID()==it->id){
                                memberExist = true;
                            }
                        }
                        if (memberExist == false){
                            memberVList.push_back(MemberV(wsm->getID(),wsm->getTcell(),wsm->getTsim(),
                                    wsm->getCsim(),wsm->getWsmData(),ttl-wsm->getTtl()));
                        }
                        EV<<"The cluster has now the below members:"<<endl;
                        for (it=memberVList.begin();it!=memberVList.end();++it){
                            EV<<"Node "<<it->id<<endl;
                        }
                    }
//                    else {
//                        state = MV;
//
//                    }
                }
            //}
            }
            break;
        }
        case QUERY_MSG:{
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
                            if (wsm->getID_Controller() == myId){
                                EV<<"Query message received from "<<wsm->getID_requester()<<endl;
                            }
                        //if (ID_Controller == wsm->getID_Controller()){
                            if ((state == GW)||(state == CH)){// Algorithm 3 and 4, line 1
                                EV << "My state is(query received) : " << state << "  and My ID is : " << myId << endl;
                                if ((myId==wsm->getID_Gateway())||(myId==wsm->getID_Controller())){
                                    EV<<"Query message received from "<<wsm->getID_requester()<<endl;
                                    requesterList.push_back(Requester(wsm->getID(),wsm->getID_requester(),// Algorithm 3 and 4, line 2
                                        wsm->getTcell(),wsm->getTsim(), wsm->getCsim(),wsm->getWsmData(),ttl-wsm->getTtl()));
                                    cRequesterList::iterator it1;
                                    for(it1=requesterList.begin();it1!=requesterList.end();++it1){
                                        EV << "requester list is node : " << it1->id << endl;
                                    }

                                    helpID = -1;
                                    bool vehicleConnected = false;
                                    bool notAllResourceAllocated = false;
                                    cConnectedVList::iterator it;
                                    for (it=connectedVList.begin();it!=connectedVList.end();++it){
                                        if((it->id != -1)&&(it->res > 0)&&(resAccess == true)){
                                            vehicleConnected = true;
                                            notAllResourceAllocated = true;
                                            helpID = it->id;
                                            break;
                                        }
                                    }
                                    if ((vehicleConnected == true)&&(notAllResourceAllocated == true)){//Algorithm 3 and 4, Line 4
                                        if (sendQueryMessage == false){
                                            ID_requester=wsm->getID_requester();
                                            sendQueryMsg();//Algorithm 3 and 4, Line 5
                                            overheadService++;
                                             requestHelpNumber++;
                                            //sendQueryMessage = true;
                                        }
                                    }
                                    else {
                                        if (wsm->getTtl() > 0){
                                            relayPacket(wsm);////Algorithm 3 and 4, Line 13
                                        }
                                    }
                                }
                            }
                            if ((state == MV)  && (res > 0)){//When a vehicle receives a query message,
                                //it verifies the availability of its resources and sends a response message with
                                //the status of the allocation to the gateway.
                                if (wsm->getRecipientAddress()!=-1){
                                    EV << "MV get but not for itself it is for : " << wsm->getRecipientAddress() << endl;
                                if(myId == wsm->getRecipientAddress()){
                                EV << "Provider is : " << myId << "  and requester is: " << wsm->getID_requester() << endl;
                                if ((resAccess == true) && (wsm->getID_requester()!=myId)){
                                    resAccess = false ;
                                    ID_requester=wsm->getID_requester();
                                    helpID = wsm->getSenderAddress();
                                    sendQueryResponseMsg();
                                    overheadService++;
                                    //responseNumber++;
                                    serviceDuration = 0.5;
                                    serviceTimer = new cMessage("serviceTimer",SERVICE_TIMER);
                                    EV<<"Transmission time is updated(RESPONSE-->CANDIDATE) : "<<transmissionTime<<endl;
                                    waitingAfterCandidateMsg = new cMessage("waitingAfterCandidateMsg",WAITING_AFTER_CANDIDATE_MSG);
                                    scheduleAt(simTime() + 5*(transmissionTime+individualOffset) + serviceDuration , serviceTimer);// n = 5;
                    }
                }
              }
             }
           }
            break;
        }
        case RESPONSE_QUERY_MSG:{// Algorithm 3 and 4, Line 7
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
            //if (ID_Controller == wsm->getID_Controller()){
                if ((state == GW)||(state == CH)){
                    EV << "My state is(response received) : " << state << "  and My ID is : " << myId << endl;
                    if (myId==wsm->getRecipientAddress()){ // Algorithm 3 and 4, Line 8 & 9
                        bool carExistGW = false;// to check if the received controlMsg
                        cConnectedVList::iterator it;                        //had been received from existed controller
                        for (it=connectedVList.begin();it!=connectedVList.end();++it){
                            if(wsm->getID()==it->id){
                                carExistGW = true;
                                connectedVList.erase(it);
                            }
                        }
                        connectedVList.push_back(ConnectedV(wsm->getID(),wsm->getTcell(),wsm->getTsim(),
                                wsm->getCsim(),ttl - wsm->getTtl(),wsm->getRes()));// Algorithm 3 and 4, Line 10

                        if (wsm->getTtl() > 0){
                            relayPacket(wsm);// Algorithm 3 and 4, Line 11
                            helpID = wsm->getID_requester();
                            sendQueryResponseMsg();
                            overheadService++;
                        }
                        else {
                            helpID = wsm->getID_requester();
                            sendQueryResponseMsg();
                            overheadService++;
                        }
                    }
                }
                if ((state == MV)&&(sendQueryMessage == true)){//Algorithm 2, Line 4
                    if (myId == wsm->getRecipientAddress()){////Algorithm 2, Line 5, 6
                        sourceID=wsm->getID();
                        EV << "source ID is : " << sourceID << endl;
                        serviceSearchingTime=simTime()-serviceSearchingTime;
                        if (serviceSearchingTime < serviceSearchingTimeMIN){
                             serviceSearchingTimeMIN = serviceSearchingTime;
                                  }
                        if(serviceSearchingTime > serviceSearchingTimeMAX){
                             serviceSearchingTimeMAX = serviceSearchingTime;
                                   }
                        useResource();//Algorithm 2, Line 7&8
                       // useResNumber++;
                    }
                }
            }
        //}
            break;
        }
        default: {
            break;
        }
    }
}
void SERVitESApplLayerOLD::handleSelfMsg(cMessage* msg){
    switch(msg->getKind()){
        case HELLO_MSG_TIMER:{// A timer to send Hello message
            if (state == ND){
            tcell = calculateTCell();
//            double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//            tcell = d/(mobility->getSpeed()+1);
            WaveShortMessage* HelloMsg = new WaveShortMessage("SERVitES",HELLO_MSG);
            t_channel channel = dataOnSch ? type_SCH : type_CCH;
            HelloMsg->addBitLength(headerLength);
            HelloMsg->addBitLength(dataLengthBits);
            switch (channel) {
                case type_SCH: HelloMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
                case type_CCH: HelloMsg->setChannelNumber(Channels::CCH); break;
            }
            for (int r = 0; r < 10; r++){
                HelloMsg->setRoute(r,-1);
            }
            HelloMsg->setRoute(0,myId);
            //EV<<"hello route is"<<HelloMsg->getRoute(0)<<endl;
            HelloMsg->setPsid(0);
            HelloMsg->setPriority(dataPriority);
            HelloMsg->setWsmVersion(1);
            HelloMsg->setTimestamp(simTime());
            HelloMsg->setSenderAddress(myId);
            HelloMsg->setRecipientAddress(-1);
            HelloMsg->setSenderPos(curPosition);
            HelloMsg->setCellNumber(getCellNumber(curPosition));
            HelloMsg->setSerial(2);
            HelloMsg->setID(myId);//ID
            HelloMsg->setRes(res);//res
            HelloMsg->setTtl(ttl);
            HelloMsg->setTraje(mobility->getRoadId()[0]);
            HelloMsg->setWsmData(mobility->getRoadId().c_str());//trajectory
            HelloMsg->setKind(HELLO_MSG);
            sendWSM(HelloMsg);// line 2
            overheadCluster++;
            waitingTimeExpireed = false;
            helloMsgSent = true;
            EV<<"Hello message is sent from "<<myId<<", with name = "<<HelloMsg->getName()<<", and kind = "<<HelloMsg->getKind()<<endl<<
                    ", at position x = "<<curPosition.x<<", y = "<<curPosition.y<<", and Traje "<<mobility->getRoadId().c_str()<<
                    ", and angel "<<mobility->getAngleRad()<<"\n";
            scheduleAt(simTime() + 5*(transmissionTime+individualOffset), waitingResponseTimer);// n = 5;
            break;
            }
        }
        case WAITING_RESPONSE_TIMER:{
            EV<<"WAITING_RESPONSE_TIMER fired"<<endl;
            waitingTimeExpireed = true;
            helloMsgSent = false;
            if (receiveMsgDuringWaiting == false){// line 30, if no response messages received during  waiting time
                EV<<"Change state to CH!"<<endl;
                state = CH; // Line 31
                ID_Controller = myId;
                sendControllerMsg(); // Line 32;
                overheadCluster++;
                joinToCluster =true;
            }
            break;
        }
        case BEACON_TIMER:{
            sendBeaconMsg();
            overheadBeacon++;
            beaconTimer = new cMessage("beaconTimer",BEACON_TIMER);
            beaconTimer->setKind(BEACON_TIMER);
            if (beaconTimer->isScheduled() != true){//After creating the cluster,
                //each vehicle inside a cluster periodically sends a beacon message to the Controller,
                scheduleAt(simTime() + beaconInterval, beaconTimer);
            }
            break;
        }
        case WAITING_AFTER_CANDIDATE_MSG:{
            EV<<"WAITING_AFTER_CANDIDATE_MSG timer fired!"<<endl;
            if (receivedControlMsgAfterCandidate == false){
                EV<<"Change state to CH!"<<endl;
                state = CH;
                ID_Controller = myId;
                sendControllerMsg();
                overheadCluster++;
                joinToCluster =true;
            }
            break;
        }
        case SERVICE_TIMER:{
            EV<<"SERVICE_TIMER fired! -- resources are free"<<endl;
            //serviceDurationFinished = true;
            resAccess = true;
            sendQueryMessage = false;
            EV << "node [" << myId <<"] resources are free -->" << resAccess << endl;
           // scheduleAt(simTime() + 5*(transmissionTime+individualOffset) + serviceDuration, serviceTimer);// n = 5;
            break;
        }
        case SERVICE_INTERVAL_TIMER:{
            if (resAccess == true){
                if(state == MV){
                    ID_requester = myId;
                    resAccess = true;
                    sendQueryMsg();
                    overheadService++;
                    sendQueryMessage = true;
                    serviceSearchingTime=simTime();
                    requestNumber++;

                   /* serviceIntervalTimer = new cMessage("serviceIntervalTimer",SERVICE_INTERVAL_TIMER);
                    serviceIntervalTimer->setKind(SERVICE_INTERVAL_TIMER);
                    if (serviceIntervalTimer->isScheduled() != true){//After creating the cluster,
                        //each vehicle inside a cluster periodically sends a service request message to the Controller,
                        scheduleAt(simTime() + serviceInterval, serviceIntervalTimer);
                    }*/
                }
            }
            else {
                EV << " we can't access to resources -- new Timer Starts..." << endl;
                if (serviceIntervalTimer->isScheduled() != true){//After creating the cluster,
                    //each vehicle inside a cluster periodically sends a service request message to the Controller,
                    scheduleAt(simTime() + serviceInterval, serviceIntervalTimer);
                      }
            }
            break;
        }
        case RESET_TIMER:{
            if(helloMsgTimer->isScheduled() != true){
                scheduleAt(simTime() + 1, helloMsgTimer);
                receiveMsgDuringWaiting = false;
                waitingTimeExpireed = false;
                helloMsgSent = false;
                candidateMsgSent = false;
                controllerMsgSent = false;
                beaconMsgSent = false;
                sendQueryMessage = false;
                resAccess = true;
                res = uniform(1,3);
                scheduleAt(simTime() + 50, resetTimer);
            }
        }
    }
}
void SERVitESApplLayerOLD::handleLowerMsg(cMessage *msg){
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);
    if (std::string(wsm->getName()) == "SERVitES") {
        onData(wsm);
    }
}

void SERVitESApplLayerOLD::sendMessage(std::string blockedRoadId) {
    sentMessage = true;

    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
}

void SERVitESApplLayerOLD::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}
void SERVitESApplLayerOLD::handleParkingUpdate(cObject* obj) {
    isParking = mobility->getParkingState();
    if (sendWhileParking == false) {
        if (isParking == true) {
            (FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
        }
        else {
            Coord pos = mobility->getCurrentPosition();
            (FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
        }
    }
}
void SERVitESApplLayerOLD::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);
    //traciVehicle->setSpeed(5);
    if (mobility->getSpeed() >= maxSpeed){
        traciVehicle->setSpeed(maxSpeed);
    }
    if (state == CH){
        CHDuration.record(1);
    }
    else {
        CHDuration.record(0);
    }
    if (resAccess == true){
        ServiceAvailability.record(1);
    }
    else {
        ServiceAvailability.record(0);
    }
    // stopped for for at least 10s?
    if (mobility->getSpeed() < 1) {
        if (simTime() - lastDroveAt >= 10) {
            findHost()->getDisplayString().updateWith("r=16,red");
            if (!sentMessage) sendMessage(mobility->getRoadId());
        }
    }
    else {
        lastDroveAt = simTime();
    }
    // some calculation to compute Avg CH duration
    counterSerial++;
    stateSerial[counterSerial] = state;
    serviceSerial[counterSerial] = resAccess;

}
void SERVitESApplLayerOLD::sendWSM(WaveShortMessage* wsm) {
    //if (isParking && !sendWhileParking) return;
    sendDelayedDown(wsm,individualOffset);
}


Coord SERVitESApplLayerOLD::GPSLocationEstimation(){
    Coord p;
    Coord gps=mobility->getCurrentPosition();
    p.x=gps.x ;
    p.y=gps.y ;
    p.z=gps.z ;
    //EV<<" Position, x = "<<pos.x<<", y = "<<pos.y<<", z = "<<pos.z<<"\n";
    EV<<"Estimated GPS position  is x = "<<p.x<<", y = "<<p.y<<", z = "<<p.z<<" \n";
    return p;
}

double SERVitESApplLayerOLD::distance(double x1,double y11,double x2,double y2,double z1,double z2){
    return (sqrt((x1-x2)*(x1-x2)+(y11-y2)*(y11-y2)+(z1-z2)*(z1-z2)));
}

int SERVitESApplLayerOLD::getCellNumber(){
    Coord p=mobility->getCurrentPosition();
    if ((p.x < xmid)&&(p.y < ymid)){
        return 1;
    }
    if ((p.x >= xmid)&&(p.y < ymid)){
        return 2;
    }
    if ((p.x < xmid)&&(p.y >= ymid)){
        return 3;
     }
    if ((p.x >= xmid)&&(p.y >= ymid)){
        return 4;
     }
}

simtime_t SERVitESApplLayerOLD::tsimCalculate(std::string road1,std::string road2){
    int similarSteps = 0;
    for (int i=0;i<9;i++){
        if (road1[i]==road2[i]){
            similarSteps++;
        }
    }
    return similarSteps*5.5;
};

void SERVitESApplLayerOLD::sendCandidateMsg(){
    WaveShortMessage* candidateMsg = new WaveShortMessage("SERVitES",CANDIDATE_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    candidateMsg->addBitLength(headerLength);
    candidateMsg->addBitLength(dataLengthBits);
    switch (channel) {
        case type_SCH: candidateMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: candidateMsg->setChannelNumber(Channels::CCH); break;
    }
    for (int r = 0; r < 10; r++){
        candidateMsg->setRoute(r,-1);
    }
    candidateMsg->setRoute(0,myId);
    candidateMsg->setPsid(0);
    candidateMsg->setPriority(dataPriority);
    candidateMsg->setWsmVersion(1);
    candidateMsg->setTimestamp(simTime());
    candidateMsg->setSenderAddress(myId);
    candidateMsg->setRecipientAddress(-1);
    candidateMsg->setSenderPos(curPosition);
    candidateMsg->setCellNumber(getCellNumber(curPosition));
    candidateMsg->setSerial(2);
    candidateMsg->setID(myId);//ID
    candidateMsg->setState(state);
    candidateMsg->setRes(res);
    candidateMsg->setTraje(mobility->getRoadId()[0]);
    candidateMsg->setWsmData(mobility->getRoadId().c_str());
    candidateMsg->setKind(CANDIDATE_MSG);
    tcell = calculateTCell();
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    candidateMsg->setTcell(tcell.dbl());//tcell
    candidateMsg->setCsim(csim);//csim
    sendWSM(candidateMsg);
    candidateMsgSent = true;
    EV<<"Candidate message sent!"<<endl;
}

void SERVitESApplLayerOLD::maintenance(){
    sendJoinMsg();
    overheadCluster++;
    sendBeaconMsg();
    overheadBeacon++;
    EV<<"Maintenance !"<<endl;

}

void SERVitESApplLayerOLD::sendControllerMsg(){
    EV<<"Sending controller message !"<<endl;
    WaveShortMessage* controllerMsg = new WaveShortMessage("SERVitES",CONTROL_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    controllerMsg->addBitLength(headerLength);
    controllerMsg->addBitLength(dataLengthBits);
    switch (channel) {
        case type_SCH: controllerMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: controllerMsg->setChannelNumber(Channels::CCH); break;
    }
    for (int r = 0; r < 10; r++){
        controllerMsg->setRoute(r,-1);
    }
    controllerMsg->setRoute(0,myId);
    controllerMsg->setPsid(0);
    controllerMsg->setPriority(dataPriority);
    controllerMsg->setWsmVersion(1);
    controllerMsg->setTimestamp(simTime());
    controllerMsg->setSenderAddress(myId);
    controllerMsg->setRecipientAddress(-1);
    controllerMsg->setSenderPos(curPosition);
    controllerMsg->setCellNumber(getCellNumber(curPosition));
    controllerMsg->setSerial(2);
    controllerMsg->setID(myId);//ID
    controllerMsg->setRes(res);
    controllerMsg->setState(state);
    controllerMsg->setTraje(mobility->getRoadId()[0]);
    controllerMsg->setWsmData(mobility->getRoadId().c_str());
    controllerMsg->setKind(CONTROL_MSG);
    tcell = calculateTCell();
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    controllerMsg->setTcell(tcell.dbl());
    controllerMsg->setCsim(csim);
    ID_Controller = myId;
    controllerMsg->setID_Controller(myId);//ID_Controllrer
    sendWSM(controllerMsg);
    controllerMsgSent = true;
    EV<<"Controller message sent!"<<endl;
}

void SERVitESApplLayerOLD::sendBeaconMsg(){
    EV<<"Sending beacon message !"<<endl;
    WaveShortMessage* beaconMsg = new WaveShortMessage("SERVitES",BEACON_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    beaconMsg->addBitLength(headerLength);
    beaconMsg->addBitLength(dataLengthBits);
    switch (channel) {
        case type_SCH: beaconMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: beaconMsg->setChannelNumber(Channels::CCH); break;
    }
    for (int r = 0; r < 10; r++){
        beaconMsg->setRoute(r,-1);
    }
    beaconMsg->setRoute(0,myId);
    beaconMsg->setPsid(0);
    beaconMsg->setPriority(dataPriority);
    beaconMsg->setWsmVersion(1);
    beaconMsg->setTimestamp(simTime());
    beaconMsg->setSenderAddress(myId);
    beaconMsg->setRecipientAddress(-1);
    beaconMsg->setSenderPos(curPosition);// coord
    beaconMsg->setCellNumber(getCellNumber(curPosition));
    beaconMsg->setSerial(2);
    beaconMsg->setID(myId);// ID
    beaconMsg->setRes(res);// res
    beaconMsg->setState(state);//state
    beaconMsg->setTraje(mobility->getRoadId()[0]);
    beaconMsg->setWsmData(mobility->getRoadId().c_str());
    beaconMsg->setKind(BEACON_MSG);
    tcell = calculateTCell();
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    beaconMsg->setTcell(tcell.dbl());//tcell
    beaconMsg->setCsim(csim);// csim
    beaconMsg->setID_Controller(ID_Controller);// ID_Controller
    sendWSM(beaconMsg);
    beaconMsgSent = true;
    EV<<"beacon message sent!"<<endl;
}
void SERVitESApplLayerOLD::relayPacket(WaveShortMessage* wsm){
    WaveShortMessage* relayMsg = new WaveShortMessage("SERVitES",CONTROL_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    relayMsg->addBitLength(headerLength);
    relayMsg->addBitLength(dataLengthBits);
    switch (channel) {
        case type_SCH: relayMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: relayMsg->setChannelNumber(Channels::CCH); break;
    }

    int hopNumber = 0;
    bool isSet = false;
    for (int r = 0; r < 10; r++){
        if ((wsm->getRoute(r) == -1)&&(isSet == false)){
            hopNumber = r;
            isSet = true;
        }
        else {
            relayMsg->setRoute(r,wsm->getRoute(r));
        }
    }
    EV<<"hopNumber is "<<hopNumber<<endl;

    relayMsg->setPsid(wsm->getPsid());
    relayMsg->setPriority(wsm->getPriority());
    relayMsg->setWsmVersion(wsm->getWsmVersion());
    relayMsg->setTimestamp(wsm->getTimestamp());
    relayMsg->setSenderAddress(wsm->getSenderAddress());
    relayMsg->setRecipientAddress(wsm->getRecipientAddress());
    relayMsg->setSenderPos(wsm->getSenderPos());
    relayMsg->setSerial(wsm->getSerial());
    relayMsg->setID(wsm->getID());
    relayMsg->setRes(wsm->getRes());
    relayMsg->setState(wsm->getState());
    relayMsg->setTraje(wsm->getTraje());
    relayMsg->setCellNumber(wsm->getCellNumber());
    relayMsg->setWsmData(wsm->getWsmData());
    relayMsg->setKind(wsm->getKind());
    relayMsg->setTcell(wsm->getTcell());
    relayMsg->setCsim(wsm->getCsim());
    relayMsg->setDestinationID(wsm->getDestinationID());
    relayMsg->setTtl(wsm->getTtl()-1);
    relayMsg->setID_Controller(wsm->getID_Controller());
    relayMsg->setTsim(wsm->getTsim());
    relayMsg->setID_requester(wsm->getID_requester());
    relayMsg->setKindGateway(wsm->getKindGateway());
    relayMsg->setID_Gateway(wsm->getID_Gateway());
    // info about the relay node
    relayMsg->setRoute(hopNumber,myId);
    sendWSM(relayMsg);
    EV<<"Message with kind "<<relayMsg->getKind()<<", and source "<<
            relayMsg->getSenderAddress()<<" has been relayed!"<<endl;
    EV<<"The route is: "<<endl;
    for (int i = 0; i <= hopNumber; i++){
        EV<<relayMsg->getRoute(i)<<" --> ";
    }
    EV<<endl;
}
void SERVitESApplLayerOLD::sendJoinMsg(){
    EV<<"Sending join message !"<<endl;
    WaveShortMessage* joinMsg = new WaveShortMessage("SERVitES",JOIN_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    joinMsg->addBitLength(headerLength);
    joinMsg->addBitLength(dataLengthBits);
    switch (channel) {
        case type_SCH: joinMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: joinMsg->setChannelNumber(Channels::CCH); break;
    }
    for (int r = 0; r < 10; r++){
        joinMsg->setRoute(r,-1);
    }
    joinMsg->setRoute(0,myId);
    joinMsg->setPsid(0);
    joinMsg->setPriority(dataPriority);
    joinMsg->setWsmVersion(1);
    joinMsg->setTimestamp(simTime());
    joinMsg->setSenderAddress(myId);
    joinMsg->setRecipientAddress(-1);
    joinMsg->setSenderPos(curPosition);// coord
    joinMsg->setCellNumber(getCellNumber(curPosition));
    joinMsg->setSerial(2);
    joinMsg->setID(myId);// ID
    joinMsg->setRes(res);// res
    joinMsg->setState(state);//state
    joinMsg->setTraje(mobility->getRoadId()[0]);
    joinMsg->setWsmData(mobility->getRoadId().c_str());
    tcell = calculateTCell();
    joinMsg->setKind(JOIN_MSG);
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    joinMsg->setTcell(tcell.dbl());//tcell
    joinMsg->setCsim(csim);// csim
    joinMsg->setID_Controller(ID_Controller);// ID_Controller
    sendWSM(joinMsg);
    EV<<"Join message sent!"<<endl;
}
void SERVitESApplLayerOLD::sendGatewayMsg(){
    EV<<"Sending gateway message !"<<endl;
    WaveShortMessage* gatewayMsg = new WaveShortMessage("SERVitES",GATEWAY_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    gatewayMsg->addBitLength(headerLength);
    gatewayMsg->addBitLength(dataLengthBits);
    switch (channel) {
       case type_SCH: gatewayMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
       case type_CCH: gatewayMsg->setChannelNumber(Channels::CCH); break;
    }
    for (int r = 0; r < 10; r++){
       gatewayMsg->setRoute(r,-1);
    }
    gatewayMsg->setRoute(0,myId);
    gatewayMsg->setPsid(0);
    gatewayMsg->setPriority(dataPriority);
    gatewayMsg->setWsmVersion(1);
    gatewayMsg->setTimestamp(simTime());
    gatewayMsg->setSenderAddress(myId);
    gatewayMsg->setRecipientAddress(-1);
    gatewayMsg->setSenderPos(curPosition);// coord
    gatewayMsg->setCellNumber(getCellNumber(curPosition));
    gatewayMsg->setSerial(2);
    gatewayMsg->setID(myId);// ID
    gatewayMsg->setRes(res);// res
    gatewayMsg->setState(state);//state
    gatewayMsg->setTraje(mobility->getRoadId()[0]);
    gatewayMsg->setWsmData(mobility->getRoadId().c_str());
    gatewayMsg->setKind(GATEWAY_MSG);
    gatewayMsg->setID_Gateway(ID_Gateway);
    gatewayMsg->setKindGateway(kindGateway);//kind
    tcell = calculateTCell();
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    gatewayMsg->setTcell(tcell.dbl());//tcell
    gatewayMsg->setCsim(csim);// csim
    gatewayMsg->setID_Controller(ID_Controller);// ID_Controller
    sendWSM(gatewayMsg);
    EV<<"Gateway message sent!"<<endl;
}

void SERVitESApplLayerOLD::sendQueryResponseMsg(){
    EV<<"Sending query response message !"<<endl;
    WaveShortMessage* responseQueryMsg = new WaveShortMessage("SERVitES",RESPONSE_QUERY_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    responseQueryMsg->addBitLength(headerLength);
    responseQueryMsg->addBitLength(dataLengthBits);
    switch (channel) {
        case type_SCH: responseQueryMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: responseQueryMsg->setChannelNumber(Channels::CCH); break;
    }
    for (int r = 0; r < 10; r++){
        responseQueryMsg->setRoute(r,-1);
    }
    responseQueryMsg->setRoute(0,myId);
    responseQueryMsg->setPsid(0);
    responseQueryMsg->setPriority(dataPriority);
    responseQueryMsg->setWsmVersion(1);
    responseQueryMsg->setTimestamp(simTime());
    responseQueryMsg->setSenderAddress(myId);
    EV << "help ID is : " << helpID << endl ;
    responseQueryMsg->setRecipientAddress(helpID);
    responseQueryMsg->setSenderPos(curPosition);// coord
    responseQueryMsg->setCellNumber(getCellNumber(curPosition));
    responseQueryMsg->setSerial(2);
    responseQueryMsg->setID(myId);// ID
    responseQueryMsg->setTtl(ttl+1);
    responseQueryMsg->setRes(res);// res
   // res = 0;
    responseQueryMsg->setID_requester(ID_requester);
    responseQueryMsg->setState(state);//state
    responseQueryMsg->setTraje(mobility->getRoadId()[0]);
    responseQueryMsg->setWsmData(mobility->getRoadId().c_str());
    responseQueryMsg->setKind(RESPONSE_QUERY_MSG);
    tcell = calculateTCell();
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    responseQueryMsg->setTcell(tcell.dbl());//tcell
    responseQueryMsg->setCsim(csim);// csim
    responseQueryMsg->setID_Controller(ID_Controller);// ID_Controller
    responseQueryMsg->setID_Gateway(ID_Gateway);
    //responseQueryMsg->setIsGateway(isGateway);
    responseQueryMsg->setResAccess(resAccess);
    sendWSM(responseQueryMsg);
    EV<<"query response message sent!"<<endl;
}

void SERVitESApplLayerOLD::sendQueryMsg(){
    EV<<"Sending Query message !"<<endl;
    WaveShortMessage* queryMsg = new WaveShortMessage("SERVitES",QUERY_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    queryMsg->addBitLength(headerLength);
    queryMsg->addBitLength(dataLengthBits);
    switch (channel) {
        case type_SCH: queryMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: queryMsg->setChannelNumber(Channels::CCH); break;
    }
    for (int r = 0; r < 10; r++){
        queryMsg->setRoute(r,-1);
    }
    queryMsg->setRoute(0,myId);
    queryMsg->setPsid(0);
    queryMsg->setPriority(dataPriority);
    queryMsg->setWsmVersion(1);
    queryMsg->setTimestamp(simTime());
    queryMsg->setSenderAddress(myId);
    EV << "help ID is : " << helpID << endl ;
    queryMsg->setRecipientAddress(helpID);
    queryMsg->setSenderPos(curPosition);// coord
    queryMsg->setCellNumber(getCellNumber(curPosition));
    queryMsg->setSerial(2);
    queryMsg->setID(myId);// ID
    queryMsg->setRes(res);// res
    queryMsg->setID_requester(ID_requester);
    EV <<"ID requester is : " << myId << endl;
    EV <<"ID requester is : " << ID_requester << endl;
    queryMsg->setState(state);//state
    queryMsg->setTraje(mobility->getRoadId()[0]);
    queryMsg->setWsmData(mobility->getRoadId().c_str());
    queryMsg->setKind(QUERY_MSG);
    tcell = calculateTCell();
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    queryMsg->setTcell(tcell.dbl());//tcell
    queryMsg->setCsim(csim);// csim
    queryMsg->setID_Controller(ID_Controller);// ID_Controller
    queryMsg->setID_Gateway(ID_Gateway);
    //queryMsg->setIsGateway(isGateway);
    queryMsg->setResAccess(resAccess);
    sendWSM(queryMsg);
    EV<<"Query message sent!"<<endl;
}
void SERVitESApplLayerOLD::useResource(){
    EV<<"Start using resources!"<<endl;
        EV << "node [" << myId << "] is using node [" << sourceID << "] resources." << endl;
        serviceDuration = 0.5;
        //serviceDurationFinished = false ;
        resAccess = false;
        serviceTimer = new cMessage("serviceTimer",SERVICE_TIMER);
        scheduleAt(simTime() + 5*(transmissionTime+individualOffset) + serviceDuration, serviceTimer);// n = 5;
}

simtime_t SERVitESApplLayerOLD::calculateTCell(){
    simtime_t T_CELL = 0;
    cellnumber = getCellNumber(curPosition);
    for (int i =0; i < 1000;i++){
        if (getCellNumber(mobility->getPositionAt(i+simTime())) == cellnumber){
            T_CELL = T_CELL + 1;
        }
        else{
            i = 999;
            break;
        }
    }
    return T_CELL;
}
int SERVitESApplLayerOLD::getCellNumber(Coord position){
    int cNumber = 0;
    if((position.x < xmid)&&(position.y >ymid)){
        cNumber = 1;
    }
    if((position.x < xmid)&&(position.y <ymid)){
        cNumber = 3;
    }
    if((position.x > xmid)&&(position.y >ymid)){
        cNumber = 2;
    }
    if((position.x > xmid)&&(position.y <ymid)){
        cNumber = 4;
    }
    return cNumber;
}

void SERVitESApplLayerOLD::finish(){
    if (sendBeaconEvt->isScheduled()) {
        cancelAndDelete(sendBeaconEvt);
    }
    else {
        delete sendBeaconEvt;
    }

    findHost()->unsubscribe(mobilityStateChangedSignal, this);
//    recordScalar("nbACKs",nbACKs);
//    recordScalar("nbNACKs",nbNACKs);
    int CHDuration = 0;
    int numCHState = 0;
    int MVDuration = 0;
    int numMVState = 0;
    int serviceAvailability = 0;
    int numserviceAvailability = 0;

    for (int i=0;i<counterSerial;i++){
        if (stateSerial[i] == CH){
            CHDuration++;
        }
        if ((stateSerial[i] != CH)&&(stateSerial[i+1] == CH)){
            numCHState++;
        }
        if (stateSerial[i] == MV){
            MVDuration++;
        }
        if ((stateSerial[i] != MV)&&(stateSerial[i+1] == MV)){
            numMVState++;
        }
    }

    for (int i=0;i<counterSerial;i++){
            if (serviceSerial[i] == true){
                serviceAvailability++;
            }
            if ((serviceSerial[i] != true)&&(serviceSerial[i+1] == true)){
                numserviceAvailability++;
            }

        }

    double AvgCHDuration = 0;
    AvgCHDuration = double(CHDuration)/double(numCHState);
    double AvgMVDuration = 0;
    AvgMVDuration = double(MVDuration)/double(numMVState);
    double AvgServiceAvailability = 0;
    AvgServiceAvailability = double(serviceAvailability)/double(numserviceAvailability);

    if (numCHState > 0){

        recordScalar("Avg CH Duration",AvgCHDuration);
    }
    else {
        recordScalar("Avg CH Duration",0);
    }
    if (numMVState > 0){

        recordScalar("Avg MV Duration",AvgMVDuration);
    }
    else {
        recordScalar("Avg MV Duration",0);
    }
    if (numserviceAvailability > 0){

           recordScalar("Avg Service Availability",serviceAvailability);
       }
    else {
           recordScalar("Avg Service Availability",0);
       }

      recordScalar("reqNumber", requestNumber);
    //recordScalar("response number",responseNumber);
    //recordScalar("use resource",useResNumber);
    recordScalar("requestHelp",requestHelpNumber);
   // recordScalar("Res Ready",resReadyNumber);
    //recordScalar("TEST VARIABLE" , testV);
    recordScalar("Service Searching TIME MIN ",serviceSearchingTimeMIN);
    recordScalar("Service Searching TIME MAX ",serviceSearchingTimeMAX);
    recordScalar("join cluster ",joinToCluster);
    recordScalar("overheadCluster", overheadCluster);
    recordScalar("overheadService", overheadService);
    recordScalar("overheadBeacon", overheadBeacon);
}
