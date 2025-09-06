/*
 * SERVitESApplLayer.cc
 *
 *  Created on: Apr 3, 2018
 *      Author: Montajab Ghanem
 */


#include "SERVitESApplLayer.h"
#include"string.h"
#include <stdio.h>
#include <stdio.h>
#include <string.h>

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t SERVitESApplLayer::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(SERVitESApplLayer);

void SERVitESApplLayer::initialize(int stage) {
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

        //speedThereshold = calculateSpeedThereshold(maxSpeed); // we must change this to mathematical model
        speedThereshold = maxSpeed/2;
        speedPenalty = 10 ; // we choose 10 because of not being negative
        res = intuniform(1,3);
        state = ND;//line 1
        resourceState = FREE ;
        counterSerial = 0;
        for (int i = 0; i< 1000; i++){
            stateSerial[i] = -1;
            GWSerial[i] = -1 ;
            serviceSerial[i] = -1;
            testNodeResponse[i]=-1;
            helpID [i]= -1;
        }
        stateSerial[counterSerial] = state;
        //serviceDuration = serviceDurationFunction(1);
        serviceDuration = 0.5;
        serviceDurationTime = 0 ;
        ID_Controller = -1;
        CSF=0;
        csim = 0;
        ttl = 0;
        hopthreshold = 1;//1
        memberNumber = 0;
        cellnumber = getCellNumber(curPosition);
        transmissionTime = 0.005;
        //transmissionTimeFactor = calculateTransmissionTimeFactor();
        beaconInterval = 2;
        serviceInterval = uniform(5,10);
        serviceSearchingTime=0;
        serviceSearchingTimeMIN=1;
        serviceSearchingTimeMAX=0;
        kindGateway = 1;// All nodes can be an internal GW
        ID_requester = -1;
        sourceID = -1;
        //helpID = -1;
        sendCnt = 0;
        requestNumber = 0;
        responseNumber = 0;
        useResNumber = 0;
        requestHelpNumber = 0;
        resReadyNumber = 0;
        //testV = 0;
        overheadCluster = 0;
        overheadService = 0;
        overheadBeacon = 0;
        memberNumber = 0;

        receiveMsgDuringWaiting = false;
        receivedControlMsgAfterCandidate = false;
        waitingTimeExpireed = false;
        helloMsgSent = false;
        candidateMsgSent = false;
        candidateMsgHelloSent = false;
        controllerMsgSent = false;
        beaconMsgSent = false;
        warningMsgSent = false;
        reorgnizingMsgSent = false;
        responseServiceMsgSent = false;
        sendQueryMessage = false;
        CHchange = false;
        isGateway = false;
        serviceDurationFinished = true;
        resAccess = true ;
        receivedResFreeMsgAfterServiceResponse = false;
        resReadyMsgSent = false;
        joinToCluster = false;
        maintenanceSetCH = false;

        CHDuration.setName("CH Duration");
        MVDuration.setName("MV Duration");
        ServiceAvailability.setName("Service Availability");
        ServiceSTATE.setName("RESOURCE STATE");
        CHnumber.setName("CH NUMBER");

        helloMsgTimer = new cMessage("HelloTimer",HELLO_MSG_TIMER);
        helloMsgTimer->setKind(HELLO_MSG_TIMER);
        scheduleAt(simTime() + 50, helloMsgTimer);

        waitingResponseTimer = new cMessage("waitingResponseTimer",WAITING_RESPONSE_TIMER);
        waitingResponseTimer->setKind(WAITING_RESPONSE_TIMER);

        resetTimer = new cMessage("resetTimer",RESET_TIMER);
       // resetTimer->setKind(RESET_TIMER);
        scheduleAt(simTime() + 100, resetTimer);

    }

}

void SERVitESApplLayer::onBeacon(WaveShortMessage* wsm) {
}

void SERVitESApplLayer::onData(WaveShortMessage* wsm) {
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
                    if ((tsimCalculate(wsm->getWsmData(),mobility->getRoadId())>0)&&(std::abs(mobility->getAngleRad()-wsm->getAngle()) <= 0.78)){
                        csim++;
                    }
                    transmissionTime = simTime() - wsm->getTimestamp();

                    EV<<"Transmission time is updated : "<<transmissionTime<<endl;
                    EV<<"Hello message has been received from "<<wsm->getID()<<endl;
                    EV<<"Path for source is "<<wsm->getWsmData()<<endl;
                    EV<<"Path   for  me  is "<<mobility->getRoadId()<<endl;
                    EV<<"I have hello Msgs from: "<<endl;
                    for (it=carList.begin();it!=carList.end();++it){
                        EV<<"Node "<<it->id<<endl;
                    }

                    tsim = tsimCalculate(wsm->getWsmData(),mobility->getRoadId());
                    tcell = calculateTCell();
                    EV<<"tsim is : "<< tsim <<", tcell is : "<< tcell << " csim is : "<< csim << " speedPenalty is : " << speedPenalty << endl;
                    CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
                    EV << "My CSF is : " << CSF << endl;

                   if (state == ND){//Line 8
                    EV << "my state (hello) = ND "<<endl;
                    if(CSF > wsm->getCSF()){
                    if (candidateMsgHelloSent == false){
                    EV<<"Condition line 10 is satisfied! (HELLO)"<<endl;
                     sendCandidateMsg();
                     overheadCluster++;
                     candidateMsgHelloSent = true;

                     //waitingAfterCandidateHelloMsg = new cMessage("waitingAfterCandidateHelloMsg",WAITING_AFTER_CANDIDATE_Hello_MSG);
                     //waitingAfterCandidateHelloMsg->setKind(WAITING_AFTER_CANDIDATE_Hello_MSG);
                     //if (waitingAfterCandidateHelloMsg->isScheduled() != true){
                     //(simTime()+5*(transmissionTime+individualOffset),waitingAfterCandidateHelloMsg);
                     //}
                        }
                           }
                    else{
                    sendResponseMsg();
                    //state = MV;
                    overheadCluster++;
                    EV<<"Response message with state '"<<state<<"' is sent to "<<wsm->getID()<<endl;
                    }
                   }

                   if((state == MV) || (state == CH)){
                      // if(ID_Controller != -1){
                           sendResponseMsg();
                           overheadCluster++;
                           EV<<"Response message with state '"<<state<<"' is sent to "<<wsm->getID()<<endl;
                       //}
                   }
                    //sendResponseMsg();
                    if (wsm->getTtl() > 0){
                        relayPacket(wsm);
                    }
                }
            }
            delete wsm;
                break;
        }
        case RESPONSE_MSG : {//Condition line 7
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
                if ((waitingTimeExpireed == false)&&(helloMsgSent == true)){// condition at line 6 of the Algorithm
                    if (receiveMsgDuringWaiting == false){ // i add this
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
                        if ((tsimCalculate(wsm->getWsmData(),mobility->getRoadId())>0)&&(std::abs(mobility->getAngleRad()-wsm->getAngle()) <= 0.78)){
                            csim++;
                        }
                    }

                    CSF=CHSelectionFactor(tcell.dbl() , csim , speedPenalty);

                    EV<<"My csim is "<<csim;
                    EV<<". Received csim is "<<wsm->getCsim()<<endl;
                    EV<<"My tcell is "<<tcell;
                    EV<<". Received tcel is "<<wsm->getTcell()<<endl;
                    EV<< "My speed penalty is : " << speedPenalty ;
                    EV<<". Received speedPenalty is "<<wsm->getSpeedPenalty()<<endl;
                    EV<< "CSF is : " << CSF ;
                    EV<<". Received CSF is "<<wsm->getCSF()<<endl;

                    EV<<"I have received responses Msgs from: "<<endl;
                    for (it=ResponsedCarList.begin();it!=ResponsedCarList.end();++it){
                        EV<<"Node "<<it->id<<endl;
                    }

                    if (wsm->getState() == ND){//Line 8
                        EV << "get state = ND "<<endl;
                        if(CSF > wsm->getCSF()){
                            if (candidateMsgSent == false){
                                EV<<"Condition line 10 is satisfied!"<<endl;
                                sendCandidateMsg();// Line 11
                                overheadCluster++;
                                candidateMsgSent = true;
                                transmissionTime = simTime() - wsm->getTimestamp();
                                transmissionTimeFactor = 5; //calculateTransmissionTimeFactor(); //5
                                EV<<"Transmission time is updated(RESPONSE-->CANDIDATE) : "<<transmissionTime<<endl;
                                waitingAfterCandidateMsg = new cMessage("waitingAfterCandidateMsg",WAITING_AFTER_CANDIDATE_MSG);
                                scheduleAt(simTime()+transmissionTimeFactor*(transmissionTime+individualOffset),waitingAfterCandidateMsg);
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
                            if (ID_Controller == -1 ){
                                EV << "CH CheckingMsg Starts..." << endl;
                                //CHcheckingMsg = new cMessage("CHcheckingMsg",CH_CHECKING_TIMER);
                                //if(!CHcheckingMsg->isScheduled()){
                                //scheduleAt(simTime()+5*(transmissionTime+individualOffset)+10,CHcheckingMsg);
                                //}
                            }
                            // Line 14 Store the information
                        }// Line 15
                    }// Line 16
                    if ((wsm->getState() == CH)||(wsm->getState() == MV)){//Line 17
                        EV << "get state (WSM) = " << wsm->getState() <<endl;
                        ID_Controller = wsm->getID_Controller();
                        if (ID_Controller != -1){
                            if (CSF <= wsm->getCSF()){
                                state = MV ;
                                CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
                                joinToCluster =true;
                                sendJoinMsg();
                                overheadCluster++;
                                maintenanceSetCH = false;
                                }
                            }
                            else {
                                receiveMsgDuringWaiting=false;
                                maintenanceSetCH = true;
                               // state = MV ;
                            }

                            EV << "Maintenance END!" << endl;
                        //maintenance();// Line 18
                    }// Line 19
                }
                if ((wsm->getDestinationID() != myId)&&(wsm->getTtl() > 0)){
                    relayPacket(wsm);
                  }
                }
            }
            delete wsm;
            break;
        }// Line 20
        case CANDIDATE_MSG:{ // Line 21
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
                if (state == ND){
                EV<<"Candidate message received from node : " << wsm->getID() <<endl;
                //cancelAndDelete(waitingResponseTimer);
                //drop(waitingResponseTimer);
                cCarList::iterator it;
                bool carExist = false;
                for (it=carList.begin();it!=carList.end();++it){
                    if((wsm->getID()==it->id)){
                        carExist = true;
                    }
                }
                if ((carExist == false)&&(wsm->getID() != myId)){
                    carList.push_back(Car(wsm->getID()));
                    if ((tsimCalculate(wsm->getWsmData(),mobility->getRoadId())>0)&&(std::abs(mobility->getAngleRad()-wsm->getAngle()) <= 0.78)){
                        csim++;
                    }
                }
                CSF=CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
                // Analysis the information, Line 22
                EV << "MY CSF is: " << CSF << " WSM CSF is: " << wsm->getCSF() << endl ;
                if(CSF > wsm->getCSF()){
                    EV<<"Change state to CH!"<<endl;
                    ID_Controller=myId;
                    state = CH; // Line 24
                    sendControllerMsg(); // Line 25;
                    overheadCluster++;
                    joinToCluster=true;
                }
                else {// Line 26
                    // Store the Information!
                    state = MV ;
                    //cancelAndDelete(waitingAfterCandidateHelloMsg);
                    //drop(waitingAfterCandidateHelloMsg);
                    ID_Controller = wsm->getID_Controller();
                    EV<<"Controller is "<<ID_Controller<<endl;
                    if (ID_Controller!=-1){
                        joinToCluster = 1;
                    }
                    if(ID_Controller == -1){
                    EV << "CH CheckingMsg Starts..." << endl;
                    //if(!CHcheckingMsg->isScheduled()){
                    //CHcheckingMsg = new cMessage("CHcheckingMsg",CH_CHECKING_TIMER);
                    //scheduleAt(simTime()+5*(transmissionTime+individualOffset)+10,CHcheckingMsg);
                    //}
                    }
                }// Line 27
               /* if (wsm->getTtl() > 0){
                    relayPacket(wsm);
                }*/
                }
            }
            delete wsm;
            break;
        }// Line 28
        case CONTROL_MSG:{
            if ((state == MV) || (state == ND)){
            bool MVJoins = false;
            EV<<"Control message received! and CSF is: " << wsm->getCSF() <<endl;
            //receivedControlMsgAfterCandidate = true;
            //cancelAndDelete(waitingAfterCandidateMsg);
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
                EV<<"Controller ID is "<<it->id <<endl;
                hcount[numberOfController] = it->hopCount;
                numberOfController++;
            }
            if (numberOfController == 1){//If there is only one CH in the vicinity, then a potential CH
                //will accept the vehicle in case their hop is lower than a predefined threshold
                if (hcount[0] < hopthreshold){
                    if (CSF <= wsm->getCSF()){
                    ID_Controller = wsm->getID_Controller();
                    state = MV;
                  //  if(CHcheckingMsg->isScheduled()){
                   // cancelEvent(CHcheckingMsg);
                  //  }
                    sendJoinMsg();
                    overheadCluster++;
                    EV<<"new MV tries to join to Cluster , CH is node :" << ID_Controller <<endl;
                    sendBeaconMsg();
                    overheadBeacon++;
                    MVJoins=true;
                    receivedControlMsgAfterCandidate = true;
                    joinToCluster = true;
                    }
                    else{
                        state = MV ;
                        EV << "wait for another controller ..." << endl;
                        // i don't accept to join to you , wait for another CH; // this can help us to reduce reorganizing msg
                    }
                }
                else {//Otherwise, the vehicle becomes a new Controller and creates a cluster.
                    EV<<"Change state to CH! -- hopThereshold is not satisfied!"<<endl;
                    ID_Controller=myId;
                    state = CH;
                    sendControllerMsg();
                    overheadCluster++;
                    receivedControlMsgAfterCandidate = true;
                    joinToCluster=true;
                }
            }
            if (numberOfController > 1){//In the event that more than one
                //cluster head exists, the vehicle joins a cluster where
                //it will remain for the greatest trajectory similarity,
                state = MV ;
                isGateway = true;
                kindGateway = 2;
                ID_Gateway = myId;
                int ID_Controller_Temp = ID_Controller;
                controllerList.sort();
                for (it=controllerList.begin();it!=controllerList.end();++it){
                ID_Controller = it->id;
                sendBeaconMsg();
                overheadBeacon++;
                }
                ID_Controller=ID_Controller_Temp;
                //vehicles that have connections with other clusters.
                //These vehicles work as a communication bridge between
                //clusters. When a vehicle succeeds in communicating with other
                //clusters, it propagates a gateway message.
                EV << " new MV: " << myId <<" becomes Gateway" << endl;
                //sendGatewayMsg();
                if (joinToCluster == false){
                    if (CSF <= wsm->getCSF()){
                    ID_Controller = wsm->getID_Controller();
                    state = MV;
                 //   if(CHcheckingMsg->isScheduled()){
                 //   cancelEvent(CHcheckingMsg);
                 //   }
                    sendJoinMsg();
                    overheadCluster++;
                    EV<<"new MV tries to join to Cluster(GW) , CH is node :" << ID_Controller <<endl;
                    sendBeaconMsg();
                    overheadBeacon++;
                    MVJoins=true;
                    receivedControlMsgAfterCandidate = true;
                    joinToCluster = true;
                              }
                    else{
                    EV << "wait for another controller(GW) ..." << endl;
                     // i don't accept to join to you , wait for another CH; // this can help us to reduce reorganizing msg
                    }
                }
            }
           /* if (wsm->getTtl() > 0){
                relayPacket(wsm);
            }*/
            if(ID_Controller != -1) {
            beaconTimer = new cMessage("beaconTimer",BEACON_TIMER);
            beaconTimer->setKind(BEACON_TIMER);
               if (beaconTimer->isScheduled() != true){//After creating the cluster,
                  //each vehicle inside a cluster periodically sends a beacon message to the Controller,
                   scheduleAt(simTime() + beaconInterval, beaconTimer);
                    }
            }

               if(MVJoins == true){
               EV << "resource number is : " << res << endl ;
               if ((res <= 2)&& (sendQueryMessage == false)){ // Algorithm 2, line 1
               serviceIntervalTimer = new cMessage("serviceIntervalTimer",SERVICE_INTERVAL_TIMER);
               serviceIntervalTimer->setKind(SERVICE_INTERVAL_TIMER);
               if (serviceIntervalTimer->isScheduled() != true){//After creating the cluster,
                      //each vehicle inside a cluster periodically sends a service request message to the Controller,
               scheduleAt(simTime() + serviceInterval, serviceIntervalTimer);
                           }
               }
               }
         }
            delete wsm;
            break;
        }

        case JOIN_MSG:{
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
            //if (ID_Controller == wsm->getID_Controller()){
                cMemberVList::iterator it;
                if ((state == CH) &&  (myId==wsm->getID_Controller())){
                    if(CSF >= wsm->getCSF()){
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
                        memberNumber = 0;
                        for (it=memberVList.begin();it!=memberVList.end();++it){
                            EV<<"Node "<<it->id<<endl;
                            memberNumber++;
                        }
                            EV << "Cluster has:" << memberNumber << " nodes " << endl;
                    }
                    else {
                        for (it=memberVList.begin();it!=memberVList.end();++it){
                               memberNumber++;
                                                }
                        EV << "controller CSF is: " << CSF << " new controller CSF is: " << wsm->getCSF() << endl ;
                        state = MV;
                        ID_Controller_Old=ID_Controller;
                        ID_Controller=wsm->getID();
                        sendWarningMsg();
                        overheadCluster++;
                        EV << "Controller is Changed from node: " << ID_Controller_Old << " to node:" << ID_Controller << endl;
                        sendReorgnizingMsg();
                        overheadCluster++;
                    }
                }
            //}
              /*  if (wsm->getTtl() > 0){
                relayPacket(wsm);
            }*/
            }
            delete wsm;
            break;
        }
        case WARNING_MSG:{
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
                if((ID_Controller == wsm->getID_Controller_Old()) && (CHchange==false)){
                    EV<<"Warning message received from "<<wsm->getID()<<endl;
                    ID_Controller = wsm->getID_Controller();
                    sendBeaconMsg();
                    overheadBeacon++;
                    CHchange =true;
                }
            }
            delete wsm;
            break;
        }

        case REORGNIZING_MSG:{
            if (getCellNumber(curPosition) == wsm->getCellNumber()){
                if(wsm->getRecipientAddress() == myId){
                   // testV++;
                    EV<<"Reorganizing message received from "<<wsm->getID()<<endl;
                    state = CH;
                    ID_Controller = myId;
                    for (int i=0 ; i <wsm->getMemberNumber() ; i++ ){
                        memberVList.push_back(MemberV(wsm->getMemberID(i),wsm->getTcell(),wsm->getTsim(),
                        wsm->getCsim(),wsm->getWsmData(),ttl-wsm->getTtl()));
                        connectedVList.push_back(ConnectedV(wsm->getMemberID(i),wsm->getTcell(),wsm->getTsim(),
                                                    wsm->getCsim(),ttl - wsm->getTtl(),wsm->getRes(),wsm->getResAccess()));
                    }

                }
            }
            delete wsm;
            break;
        }

        case BEACON_MSG:{
                    if (getCellNumber(curPosition) == wsm->getCellNumber()){
                        if ((isGateway == true)||(state == CH)){
                            if((ID_Controller==wsm->getID_Controller()) || (ID_Gateway==wsm->getID_Gateway())){ // I add this line to check whether is belong to controller or not
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
                                    wsm->getCsim(),ttl - wsm->getTtl(),wsm->getRes(), wsm->getResAccess()));
                        }
                       }
                    }
                    delete wsm;
                    break;
                }

        case QUERY_MSG:{
            //if (getCellNumber(curPosition) == wsm->getCellNumber()){
                EV << "im here1" << endl;
                if ((isGateway == true)||(state == CH)){// Algorithm 3 and 4, line 1
                    EV << " im here 2" << endl;
                    if(myId == wsm->getRecipientAddress()){
                        EV << "Query MSG which can't find Resources is Received" << endl;
                    cConnectedVList::iterator it;
                    int i = 1;
                    for (it=connectedVList.begin();it!=connectedVList.end();++it){
                        if(it->resAccess == true){
                            helpID[i] = it->id;
                            i++;
                        }
                        //break;
                    }
                    if (helpID[requestHelpNumber+1] != -1){
                        ID_requester = wsm->getID_requester();
                        EV << "New Query Msg Send to " << helpID[requestHelpNumber+1] << endl;
                        helpID[0]=helpID[requestHelpNumber+1];
                        sendQueryMsg();
                        overheadService++;
                        requestHelpNumber++;
                    }
                    else {
                        if (wsm->getTtl() > 0){
                            relayPacket(wsm);
                        }
                    }
                  }

                }
                    if ((myId == wsm->getRecipientAddress()) && (ID_Controller != wsm->getRecipientAddress())){
                        EV << "Provider NEW is : " << myId << " resAccess is : "<< resAccess<< "  and requester is: " << wsm->getID_requester() <<" resAccess is: " << resAccess << endl;
                        if(resAccess == true){
                            resAccess = false ;
                            ID_requester=wsm->getID_requester();
                            serviceDurationTime = wsm->getServiceDuration();
                            sendQueryResponseMsg();
                            overheadService++;
                            responseServiceMsgSent = true;
                            responseNumber++;
                            resourceState = BUSY; // check with busy and free
                            transmissionTime = simTime() - wsm->getTimestamp();
                            transmissionTimeFactor = 10; // calculateTransmissionTimeFactor();//10
                            EV<<"Transmission time is updated(QUERY-->RESPONSE) : "<<transmissionTime<<endl;
                            waitingAfterServiceResponseMsg = new cMessage("waitingAfterServiceResponseMsg",WAITING_AFTER_SERVICE_RESPONSE_MSG);
                            scheduleAt(simTime()+transmissionTimeFactor*(transmissionTime+individualOffset),waitingAfterServiceResponseMsg);

                        }
                    }
                    if (((state == MV) || (isGateway == true)) && (res > 0)){//When a vehicle receives a query message,
                          //it verifies the availability of its resources and sends a response message with
                          //the status of the allocation to the gateway.
                        EV << "Provider is : " << myId << " resAccess is : "<< resAccess<< "  and requester is: " << wsm->getID_requester() <<" resAccess is: " << resAccess << endl;
                        if ((resAccess == true) && (wsm->getID_requester()!=myId)){
                        resAccess = false ;
                        ID_requester=wsm->getID_requester();
                        serviceDurationTime = wsm->getServiceDuration();
                        sendQueryResponseMsg();
                        overheadService++;
                        responseServiceMsgSent = true;
                        responseNumber++;
                        resourceState = BUSY; // check with busy and free
                        transmissionTime = simTime() - wsm->getTimestamp();
                        transmissionTimeFactor = 10; // calculateTransmissionTimeFactor();//10
                        EV<<"Transmission time is updated(QUERY-->RESPONSE) : "<<transmissionTime<<endl;
                        waitingAfterServiceResponseMsg = new cMessage("waitingAfterServiceResponseMsg",WAITING_AFTER_SERVICE_RESPONSE_MSG);
                        scheduleAt(simTime()+transmissionTimeFactor*(transmissionTime+individualOffset),waitingAfterServiceResponseMsg);
                                        }
                       /* if (wsm->getTtl() > 0){
                            relayPacket(wsm);
                        }*/
              }
            //}
            delete wsm;
            break;
        }
        case RESPONSE_QUERY_MSG:{// Algorithm 3 and 4, Line 7
           // if (getCellNumber(curPosition) == wsm->getCellNumber()){
                if ((isGateway == true)||(state == CH)){
                    if (wsm->getRes() > 0){ // Algorithm 3 and 4, Line 8 & 9
                        bool carExistGW = false;// to check if the received controlMsg
                        cConnectedVList::iterator it;                        //had been received from existed controller
                        for (it=connectedVList.begin();it!=connectedVList.end();++it){
                            if(wsm->getID()==it->id){
                                carExistGW = true;
                                connectedVList.erase(it);
                            }
                        }
                        connectedVList.push_back(ConnectedV(wsm->getID(),wsm->getTcell(),wsm->getTsim(),
                                wsm->getCsim(),ttl - wsm->getTtl(),wsm->getRes(),wsm->getResAccess()));// Algorithm 3 and 4, Line 10
                        if (wsm->getTtl() > 0){
                            relayPacket(wsm);// Algorithm 3 and 4, Line 11
                        }
                    }
                }
               // if ((state == MV)&&(sendQueryMessage == true)){
                    //EV << "i get into 1" << endl;
                    if (myId == wsm->getID_requester()){
                        if(resReadyMsgSent == false){
                        EV << " my id is: " << myId << "res ready sent to:" << wsm->getSenderAddress() << endl;
                        ID_requester = myId;
                        sourceID = wsm->getSenderAddress();
                        resourceState = BUSY;
                        serviceSearchingTime=simTime()-serviceSearchingTime;
                        if (serviceSearchingTime < serviceSearchingTimeMIN){
                            serviceSearchingTimeMIN = serviceSearchingTime;
                        }
                        if(serviceSearchingTime > serviceSearchingTimeMAX){
                            serviceSearchingTimeMAX = serviceSearchingTime;
                        }
                        sendResReadyMsg();
                        overheadService++;
                        resReadyNumber++;
                        //resReadyMsgSent = true;
                        serviceDurationTime = serviceDuration;
                        transmissionTime = simTime() - wsm->getTimestamp();
                        //transmissionTimeFactor = calculateTransmissionTimeFactor();//0.2 - 6
                        EV<<"Transmission time is updated(RESPONSE-->RES_READY) : "<<transmissionTime<<endl;
                        waitingAfterResReadyMsg = new cMessage("waitingAfterResReadyMsg",WAITING_AFTER_RES_READY_MSG);
                        scheduleAt(simTime()+0.2*(transmissionTime+individualOffset),waitingAfterResReadyMsg);
                        serviceTimer = new cMessage("serviceTimer",SERVICE_TIMER);
                        scheduleAt(simTime() + 6*(transmissionTime+individualOffset) + serviceDurationTime , serviceTimer);// n = 5;
                        }
                    }
                  /*   if (wsm->getTtl() > 0){
                        relayPacket(wsm);
                    }*/
               // }
            //}
            delete wsm;
            break;
        }
        case RES_READY_MSG:{
                    //if (getCellNumber(curPosition) == wsm->getCellNumber()){
                        if ((myId == wsm->getRecipientAddress())&&(responseServiceMsgSent == true)){

                                EV << "RES_READY_MSG is received" << endl;
                                ID_requester = wsm->getID_requester();
                                receivedResFreeMsgAfterServiceResponse = true;
                                serviceDurationTime = wsm->getServiceDuration();
                                //serviceSearchingTime=simTime()-serviceSearchingTime;
                                EV << "Time of Service Duration is : " << serviceDurationTime << " s -- MY is: " << serviceDuration << endl;
                                useResource();
                                useResNumber++;
                      }
                 //  }
                    delete wsm;
                    break;
                }
        default: {
            delete wsm;
            break;
        }
    }
}
void SERVitESApplLayer::handleSelfMsg(cMessage* msg){
    switch(msg->getKind()){
        case HELLO_MSG_TIMER:{// A timer to send Hello message
            if(state == ND) {
            sendHelloMsg();
            overheadCluster++;
            waitingTimeExpireed = false;
            helloMsgSent = true;
            EV<<"Hello message is sent from "<<myId<<endl<<
                    ", at position x = "<<curPosition.x<<", y = "<<curPosition.y<<", and Traje "<<mobility->getRoadId().c_str()<<
                    ", and angel "<<mobility->getAngleRad()<<"\n";
            transmissionTimeFactor = 5; // calculateTransmissionTimeFactor();//5
            scheduleAt(simTime() + transmissionTimeFactor*(transmissionTime+individualOffset), waitingResponseTimer);// n = 5;
            }
            //delete msg;
            break;
        }
        case WAITING_RESPONSE_TIMER:{

            EV<<"WAITING_RESPONSE_TIMER fired"<<endl;
            waitingTimeExpireed = true;
            helloMsgSent = false;
            if (receiveMsgDuringWaiting == false){// line 30, if no response messages received during  waiting time
               // if(state == ND){
                if(maintenanceSetCH == false){
                EV<<"Change state to CH! -- no response Msg received!"<<endl;
                state = CH; // Line 31
                ID_Controller = myId;
                sendControllerMsg(); // Line 32;
                overheadCluster++;
                joinToCluster=true;
            }
        }
            //delete msg;
            break;
        }
        case BEACON_TIMER:{
            if(state!=CH){
            sendBeaconMsg();
            overheadBeacon++;
            beaconTimer = new cMessage("beaconTimer",BEACON_TIMER);
            beaconTimer->setKind(BEACON_TIMER);
            if (beaconTimer->isScheduled() != true){//After creating the cluster,
                //each vehicle inside a cluster periodically sends a beacon message to the Controller,
                scheduleAt(simTime() + beaconInterval, beaconTimer);
            }
        }
            //delete msg;
            break;
        }
        case WAITING_AFTER_CANDIDATE_MSG:{
            EV<<"WAITING_AFTER_CANDIDATE_MSG timer fired!"<<endl;
            //candidateMsgSent = false;
            if (receivedControlMsgAfterCandidate == false){
                EV<<"Change state to CH! -- after candidate MSG"<<endl;
                ID_Controller=myId;
                state = CH;
                sendControllerMsg();
                overheadCluster++;
                joinToCluster=true;
            }
            //delete msg;
            break;
        }

        case WAITING_AFTER_CANDIDATE_Hello_MSG:{
                    EV<<"WAITING_AFTER_CANDIDATE_Hello_MSG timer fired!"<<endl;
                    //candidateMsgHelloSent = false;
                    if (receivedControlMsgAfterCandidate == false){
                        EV<<"Change state to CH! -- after candidate MSG"<<endl;
                        ID_Controller=myId;
                        state = CH;
                        sendControllerMsg();
                        joinToCluster=true;
                    }
                    //delete msg;
                    break;
                }
        case CH_CHECKING_TIMER:{
                    EV<<"CH_CHECKING timer fired!"<<endl;
                    if (ID_Controller == -1){
                        EV<<"No Controller Assigned! -- change to CH"<<endl;
                        ID_Controller=myId;
                        state = CH;
                        sendControllerMsg();
                        overheadCluster++;
                        joinToCluster=true;
                    }
                   // delete msg;
                    break;
                }
        case WAITING_AFTER_SERVICE_RESPONSE_MSG:{
                   EV<<"WAITING_AFTER_SERVICE_RESPONSE_MSG timer fired!"<<endl;
                   if (receivedResFreeMsgAfterServiceResponse == false){
                       EV<<" no Allocation ! -- node[" << myId << "] is free!" <<endl;
                       resourceState = FREE;
                       resAccess = true ;

                   }
                   else {
                       EV << "node [" << myId <<"] is used..." << endl;
                       resourceState = ALLOCATE;
                       resAccess = false ;
                       receivedResFreeMsgAfterServiceResponse = false;
                   }
                   //delete msg;
                   break;
                   }
        case WAITING_AFTER_RES_READY_MSG:{
                   EV<<"WAITING_AFTER_RES_READY_MSG timer fired!"<<endl;
                   if (resAccess == false){
                       EV<<" node[" << myId << "] is using other resources!" <<endl;
                       resourceState = ALLOCATE;
                       resReadyMsgSent = true;

                   }
                   else {
                       EV << "node [" << myId <<"] is free" << endl;
                       //resourceState = FREE;
                       //resAccess = true ;
                   }
                   //delete msg;
                   break;
                   }
        case WAITING_AFTER_QUERY_MSG:{
                          EV<<"WAITING_AFTER_QUERY_MSG timer fired!"<<endl;
                          if (resReadyMsgSent == false){
                              EV<<" node[" << myId << "] can't find any resources" <<endl;
                              ID_requester = myId;
                              resAccess = false;
                              resourceState = SEARCHING;
                              int random = intuniform(0,1);

                              if(random == 0){
                              helpID[0] = ID_Controller;
                              }
                              else{
                              helpID[0] = ID_Gateway;
                              }
                              if(sendCnt > 2){
                                  transmissionTimeFactor = 10; // calculateTransmissionTimeFactor();//10
                                  waitingAfterQueryMsg = new cMessage(" waitingAfterQueryMsg",WAITING_AFTER_QUERY_MSG);
                                  scheduleAt(simTime()+transmissionTimeFactor*(transmissionTime+individualOffset)+10,waitingAfterQueryMsg);
                                  sendCnt = 0;
                                  break;
                              }
                              else{
                              sendCnt++ ;
                              }
                              EV << "ID_Gateway is :" << ID_Gateway << endl;
                              sendQueryMsg();
                              overheadService++;
                              requestHelpNumber++;
                              sendQueryMessage = true;
                              serviceSearchingTime=simTime();
                              transmissionTimeFactor = 10; // calculateTransmissionTimeFactor();//10
                              waitingAfterQueryMsg = new cMessage(" waitingAfterQueryMsg",WAITING_AFTER_QUERY_MSG);
                              scheduleAt(simTime()+transmissionTimeFactor*(transmissionTime+individualOffset),waitingAfterQueryMsg);
                          }
                          else {
                              EV << "node [" << myId <<"] find some resources! -- process is being continued" << endl;
                              //resourceState = FREE;
                              //resAccess = true ;
                              sendCnt = 0;
                          }
                          //delete msg;
                          break;
                          }
        case SERVICE_TIMER:{
            EV<<"SERVICE_TIMER fired! -- resources are free"<<endl;
            serviceDurationFinished = true;
            resAccess = true;
            resourceState = FREE;
            sendQueryMessage = false;
            resReadyMsgSent = false;
            responseServiceMsgSent=false;
            receivedResFreeMsgAfterServiceResponse=false;
            EV << "node [" << myId <<"] resources are free -->" << resAccess << endl;
           // scheduleAt(simTime() + 5*(transmissionTime+individualOffset) + serviceDuration, serviceTimer);// n = 5;
            //delete msg;
            break;
        }
        case SERVICE_INTERVAL_TIMER:{
            if (resAccess == true){
                if((state == MV)&&(isGateway == false)){
                    ID_requester = myId;
                    resAccess = false;
                    resourceState = SEARCHING;
                    sendQueryMsg();
                    overheadService++;
                    sendQueryMessage = true;
                    serviceSearchingTime=simTime();
                    requestNumber++;
                    transmissionTimeFactor = 10; // calculateTransmissionTimeFactor();//10

                    waitingAfterQueryMsg = new cMessage(" waitingAfterQueryMsg",WAITING_AFTER_QUERY_MSG);
                    scheduleAt(simTime()+transmissionTimeFactor*(transmissionTime+individualOffset),waitingAfterQueryMsg);

                /*    serviceIntervalTimer = new cMessage("serviceIntervalTimer",SERVICE_INTERVAL_TIMER);
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
            //delete msg;
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
                warningMsgSent = false;
                sendQueryMessage = false;
                responseServiceMsgSent = false;
                receivedResFreeMsgAfterServiceResponse = false;
                resReadyMsgSent = false;
                CHchange = false;
                resAccess = true;
                res = intuniform(1,3);
                //serviceDurationFinished = true;
                scheduleAt(simTime() + 50, resetTimer);
            }
        }
    }
}
void SERVitESApplLayer::handleLowerMsg(cMessage *msg){
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);
    if (std::string(wsm->getName()) == "SERVitES") {
        onData(wsm);
    }
}

void SERVitESApplLayer::sendMessage(std::string blockedRoadId) {
    sentMessage = true;

    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
}

void SERVitESApplLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}
void SERVitESApplLayer::handleParkingUpdate(cObject* obj) {
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
void SERVitESApplLayer::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);

    if (mobility->getSpeed() >= maxSpeed){
        traciVehicle->setSpeed(maxSpeed);
    }

    speedPenalty = speedPenaltyFunction(mobility->getSpeed());
    EV << "now ! speedPenalty is: " << speedPenalty <<endl;

   /* double testTransmissionFactor;
    testTransmissionFactor = calculateTransmissionTimeFactor();
    EV << "now ! TransmissionFactor is: " << testTransmissionFactor <<endl;*/
    EV<<" position x = "<<curPosition.x<<", y = "<<curPosition.y << endl;

    if (state == CH){
        CHDuration.record(1);
    }
    else {
        CHDuration.record(0);
    }

    if (state == MV){
        MVDuration.record(1);
    }
    else {
        MVDuration.record(0);
    }

    if (resAccess == true){
        ServiceAvailability.record(1);
    }
    else {
        ServiceAvailability.record(0);
    }

    if (resourceState == FREE){
        ServiceSTATE.record(0);
    }
    if (resourceState == SEARCHING ){
        ServiceSTATE.record(1);
    }
    if (resourceState == BUSY){
        ServiceSTATE.record(2);
    }
    if (resourceState == ALLOCATE){
        ServiceSTATE.record(3);
    }

    if (ID_Controller == -1){
        CHnumber.record(0);
    }
    else {
        CHnumber.record(1);
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
    GWSerial[counterSerial] = isGateway;
    serviceSerial[counterSerial] = resAccess;

}
void SERVitESApplLayer::sendWSM(WaveShortMessage* wsm) {
    //if (isParking && !sendWhileParking) return;
    sendDelayedDown(wsm,individualOffset);
}


Coord SERVitESApplLayer::GPSLocationEstimation(){
    Coord p;
    Coord gps=mobility->getCurrentPosition();
    p.x=gps.x ;
    p.y=gps.y ;
    p.z=gps.z ;
    //EV<<" Position, x = "<<pos.x<<", y = "<<pos.y<<", z = "<<pos.z<<"\n";
    EV<<"Estimated GPS position  is x = "<<p.x<<", y = "<<p.y<<", z = "<<p.z<<" \n";
    return p;
}

double SERVitESApplLayer::distance(double x1,double y11,double x2,double y2,double z1,double z2){
    return (sqrt((x1-x2)*(x1-x2)+(y11-y2)*(y11-y2)+(z1-z2)*(z1-z2)));
}

int SERVitESApplLayer::getCellNumber(){
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

simtime_t SERVitESApplLayer::tsimCalculate(std::string road1,std::string road2){
    int similarSteps = 0;
    for (int i=0;i<9;i++){
        if (road1[i]==road2[i]){
            similarSteps++;
        }
    }
    return similarSteps*5.5;
};
void SERVitESApplLayer::sendHelloMsg(){
    tcell = calculateTCell();
    CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
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
    HelloMsg->setResAccess(resAccess);
    CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
    HelloMsg->setCSF(CSF);
    HelloMsg->setKind(HELLO_MSG);
    sendWSM(HelloMsg);// line 2

}
void SERVitESApplLayer::sendResponseMsg(){
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
    ResponseMsg->setDestinationID(-1);
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
    ResponseMsg->setSpeedPenalty(speedPenalty);//speedPenalty
    ResponseMsg->setCSF(CSF);
    ResponseMsg->setKind(RESPONSE_MSG);
    ResponseMsg->setResAccess(resAccess);
    sendWSM(ResponseMsg);// line 4
    EV<<"Response message sent!"<<endl;
}
void SERVitESApplLayer::sendCandidateMsg(){
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
    CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
    candidateMsg->setCSF(CSF);
    candidateMsg->setResAccess(resAccess);
    sendWSM(candidateMsg);
    //candidateMsgSent = true;
    EV<<"Candidate message sent!"<<endl;
}

void SERVitESApplLayer::sendControllerMsg(){
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
    CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
    controllerMsg->setCSF(CSF);
    ID_Controller = myId;
    controllerMsg->setID_Controller(myId);//ID_Controllrer
    controllerMsg->setResAccess(resAccess);
    sendWSM(controllerMsg);
    controllerMsgSent = true;
    EV<<"Controller message sent!"<<endl;
}

void SERVitESApplLayer::sendBeaconMsg(){
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
    CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
    beaconMsg->setCSF(CSF);
    beaconMsg->setID_Controller(ID_Controller);// ID_Controller
    beaconMsg->setResAccess(resAccess);
    beaconMsg->setIsGateway(isGateway);
    beaconMsg->setID_Gateway(ID_Gateway);
    sendWSM(beaconMsg);
    beaconMsgSent = true;
    EV<<"beacon message sent to "<< ID_Controller <<endl;
}
void SERVitESApplLayer::sendWarningMsg(){
    EV<<"Sending Warning message !"<<endl;
    WaveShortMessage* warningMsg = new WaveShortMessage("SERVitES",WARNING_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    warningMsg->addBitLength(headerLength);
    warningMsg->addBitLength(dataLengthBits);
    switch (channel) {
        case type_SCH: warningMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: warningMsg->setChannelNumber(Channels::CCH); break;
    }
    for (int r = 0; r < 10; r++){
        warningMsg->setRoute(r,-1);
    }
    warningMsg->setRoute(0,myId);
    warningMsg->setPsid(0);
    warningMsg->setPriority(dataPriority);
    warningMsg->setWsmVersion(1);
    warningMsg->setTimestamp(simTime());
    warningMsg->setSenderAddress(myId);
    warningMsg->setRecipientAddress(-1);
    warningMsg->setSenderPos(curPosition);// coord
    warningMsg->setCellNumber(getCellNumber(curPosition));
    warningMsg->setSerial(2);
    warningMsg->setID(myId);// ID
    warningMsg->setRes(res);// res
    warningMsg->setState(state);//state
    warningMsg->setTraje(mobility->getRoadId()[0]);
    warningMsg->setWsmData(mobility->getRoadId().c_str());
    warningMsg->setKind(WARNING_MSG);
    tcell = calculateTCell();
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    warningMsg->setTcell(tcell.dbl());//tcell
    warningMsg->setCsim(csim);// csim
    CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
    warningMsg->setCSF(CSF);
    warningMsg->setID_Controller(ID_Controller);// ID_Controller
    warningMsg->setID_Controller_Old(ID_Controller_Old);
    warningMsg->setResAccess(resAccess);
    sendWSM(warningMsg);
    warningMsgSent = true;
    EV<<"Warning message sent!"<<endl;
}
void SERVitESApplLayer::sendReorgnizingMsg(){
    EV<<"Sending Reorgnizing message !"<<endl;
    WaveShortMessage* reorgnizingMsg = new WaveShortMessage("SERVitES",WARNING_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    reorgnizingMsg->addBitLength(headerLength);
    reorgnizingMsg->addBitLength(dataLengthBits);
    switch (channel) {
        case type_SCH: reorgnizingMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: reorgnizingMsg->setChannelNumber(Channels::CCH); break;
    }
    for (int r = 0; r < 10; r++){
        reorgnizingMsg->setRoute(r,-1);
    }
    reorgnizingMsg->setRoute(0,myId);
    reorgnizingMsg->setPsid(0);
    reorgnizingMsg->setPriority(dataPriority);
    reorgnizingMsg->setWsmVersion(1);
    reorgnizingMsg->setTimestamp(simTime());
    reorgnizingMsg->setSenderAddress(myId);
    reorgnizingMsg->setRecipientAddress(ID_Controller);
    reorgnizingMsg->setSenderPos(curPosition);// coord
    reorgnizingMsg->setCellNumber(getCellNumber(curPosition));
    reorgnizingMsg->setSerial(2);
    reorgnizingMsg->setID(myId);// ID
    reorgnizingMsg->setRes(res);// res
    reorgnizingMsg->setState(state);//state
    reorgnizingMsg->setTraje(mobility->getRoadId()[0]);
    reorgnizingMsg->setWsmData(mobility->getRoadId().c_str());
    reorgnizingMsg->setKind(REORGNIZING_MSG);
    tcell = calculateTCell();
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    reorgnizingMsg->setTcell(tcell.dbl());//tcell
    reorgnizingMsg->setCsim(csim);// csim
    CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
    reorgnizingMsg->setCSF(CSF);
    reorgnizingMsg->setID_Controller(ID_Controller);// ID_Controller
    reorgnizingMsg->setID_Controller_Old(ID_Controller_Old);
    reorgnizingMsg->setMemberNumber(memberNumber);
    for(int i=0 ; i < memberNumber ; i++){
        reorgnizingMsg->setMemberID(i,-1);
    }
    int cnt = 0;
    cMemberVList::iterator it;
    for (it=memberVList.begin();it!=memberVList.end();++it){
        reorgnizingMsg->setMemberID(cnt,it->id);
        cnt++;
    }
    reorgnizingMsg->setResAccess(resAccess);
    sendWSM(reorgnizingMsg);
    reorgnizingMsgSent = true;
    EV<<"Reorgnizing message sent!"<<endl;
}
void SERVitESApplLayer::relayPacket(WaveShortMessage* wsm){
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
    relayMsg->setResAccess(resAccess);
    sendWSM(relayMsg);
    EV<<"Message with kind "<<relayMsg->getKind()<<", and source "<<
            relayMsg->getSenderAddress()<<" has been relayed!"<<endl;
    EV<<"The route is: "<<endl;
    for (int i = 0; i <= hopNumber; i++){
        EV<<relayMsg->getRoute(i)<<" --> ";
    }
    EV<<endl;
}
void SERVitESApplLayer::sendJoinMsg(){
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
    //CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
    joinMsg->setCSF(CSF);
    joinMsg->setID_Controller(ID_Controller);// ID_Controller
    joinMsg->setResAccess(resAccess);
    sendWSM(joinMsg);
    EV<<"Join message sent!"<<endl;
}
void SERVitESApplLayer::sendGatewayMsg(){
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
    gatewayMsg->setIsGateway(isGateway);
    gatewayMsg->setResAccess(resAccess);
    sendWSM(gatewayMsg);
    EV<<"Gateway message sent!"<<endl;
}

void SERVitESApplLayer::sendQueryResponseMsg(){
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
    responseQueryMsg->setRecipientAddress(-1);
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
    responseQueryMsg->setIsGateway(isGateway);
    responseQueryMsg->setResAccess(resAccess);
    responseQueryMsg->setServiceDuration(serviceDurationTime);
    sendWSM(responseQueryMsg);
    EV<<"query response message sent!"<<endl;
}

void SERVitESApplLayer::sendQueryMsg(){
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
    EV << "help id is (RecipeintAddress) : " << helpID[0] << endl;
    queryMsg->setRecipientAddress(helpID[0]);
    queryMsg->setSenderPos(curPosition);// coord
    queryMsg->setCellNumber(getCellNumber(curPosition));
    queryMsg->setSerial(2);
    queryMsg->setID(myId);// ID
    queryMsg->setRes(res);// res
    queryMsg->setID_requester(ID_requester);
    EV <<"ID requester is(my ID) : " << myId << endl;
    EV <<"ID requester is : " << ID_requester << endl;
    queryMsg->setState(state);//state
    EV << "state is (query) : " << state << endl;
    queryMsg->setTraje(mobility->getRoadId()[0]);
    queryMsg->setWsmData(mobility->getRoadId().c_str());
    queryMsg->setKind(QUERY_MSG);
    tcell = calculateTCell();
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    queryMsg->setTcell(tcell.dbl());//tcell
    queryMsg->setCsim(csim);// csim
    queryMsg->setID_Controller(ID_Controller);// ID_Controller
    queryMsg->setIsGateway(isGateway);
    queryMsg->setResAccess(resAccess);
    queryMsg->setServiceDuration(serviceDuration);
    sendWSM(queryMsg);
    EV<<"Query message sent!"<<endl;
}

void SERVitESApplLayer::sendResReadyMsg(){
    EV<<"Sending RES READY message !"<<endl;
    WaveShortMessage* resReadyMsg = new WaveShortMessage("SERVitES",RES_READY_MSG);
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    resReadyMsg->addBitLength(headerLength);
    resReadyMsg->addBitLength(dataLengthBits);
    switch (channel) {
        case type_SCH: resReadyMsg->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        case type_CCH: resReadyMsg->setChannelNumber(Channels::CCH); break;
    }
    for (int r = 0; r < 10; r++){
        resReadyMsg->setRoute(r,-1);
    }
    resReadyMsg->setRoute(0,myId);
    resReadyMsg->setPsid(0);
    resReadyMsg->setPriority(dataPriority);
    resReadyMsg->setWsmVersion(1);
    resReadyMsg->setTimestamp(simTime());
    resReadyMsg->setSenderAddress(myId);
    resReadyMsg->setRecipientAddress(sourceID);
    resReadyMsg->setSenderPos(curPosition);// coord
    resReadyMsg->setCellNumber(getCellNumber(curPosition));
    resReadyMsg->setSerial(2);
    resReadyMsg->setID(myId);// ID
    resReadyMsg->setRes(res);// res
    resReadyMsg->setID_requester(ID_requester);
    EV <<"ID requester is : " << myId << endl;
    EV <<"ID requester is : " << ID_requester << endl;
    resReadyMsg->setState(state);//state
    resReadyMsg->setTraje(mobility->getRoadId()[0]);
    resReadyMsg->setWsmData(mobility->getRoadId().c_str());
    resReadyMsg->setKind(RES_READY_MSG);
    tcell = calculateTCell();
//    double d = distance(curPosition.x,curPosition.y,xmid,ymid,0,0);
//    tcell = d/(mobility->getSpeed()+1);
    resReadyMsg->setTcell(tcell.dbl());//tcell
    resReadyMsg->setCsim(csim);// csim
    resReadyMsg->setID_Controller(ID_Controller);// ID_Controller
    resReadyMsg->setIsGateway(isGateway);
    resReadyMsg->setResAccess(resAccess);
    resReadyMsg->setServiceDuration(serviceDuration);
    sendWSM(resReadyMsg);
    EV<<"RES READY message sent!"<<endl;
}

void SERVitESApplLayer::maintenance(){
    EV<<"Maintenance !"<<endl;
 /*   if (CSF <= wsm->getCSF){
        state = MV;
        sendJoinMsg();
    }
    else {
        state = MV ;
    } */
    if (ID_Controller != -1){
        state = MV ;
        CSF = CHSelectionFactor(tcell.dbl() , csim , speedPenalty);
        joinToCluster =true;
        sendJoinMsg();
        overheadCluster++;
        maintenanceSetCH = false;
    }
    else {
        receiveMsgDuringWaiting=false;
        maintenanceSetCH = true;
       // state = MV ;
    }

    EV << "Maintenance END!" << endl;
}

void SERVitESApplLayer::useResource(){
    EV<<"Start using resources!"<<endl;
    EV << "node [" << ID_requester << "] is using node [" << myId << "] resources." << endl;
    //serviceDuration = serviceDurationFunction(1);
    serviceDurationFinished = false ;
    resAccess = false;
    resourceState = ALLOCATE;
    serviceTimer = new cMessage("serviceTimer",SERVICE_TIMER);
    transmissionTimeFactor = 5; // calculateTransmissionTimeFactor();//5
    scheduleAt(simTime() + transmissionTimeFactor*(transmissionTime+individualOffset) + serviceDurationTime, serviceTimer);// n = 5;

}
simtime_t SERVitESApplLayer::serviceDurationFunction(int resRequest){

    simtime_t SD;

    if(resRequest == 1){
        //SD = 0.5;
        SD = uniform(1,5);
    }
    if(resRequest == 2){
        //SD = 0.5;
        SD = uniform(1,5);
    }
    if(resRequest == 3){
        //SD = 0.5;
        SD = uniform(1,5);
    }
    return SD;
}

simtime_t SERVitESApplLayer::calculateTCell(){
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
int SERVitESApplLayer::getCellNumber(Coord position){
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

double SERVitESApplLayer::speedPenaltyFunction(double speed){

    if ((speedThereshold-0.2*speedThereshold<speed) && (speed < speedThereshold+0.2*speedThereshold)){
        speedPenalty=speedPenalty + delta;
    }
    else
    {
        speedPenalty=speedPenalty - delta;
    }
return speedPenalty ;
}
double SERVitESApplLayer::CHSelectionFactor (double CellTime , int VehicleNumber , double SpeedFactor ){
    /*if ((CellTime >= 0) && (CellTime < 10)) {
    return alpha*CellTime + beta*VehicleNumber + gama*SpeedFactor;
    }
    if ((CellTime >= 10) && (CellTime < 100)) {
    return alpha*CellTime/10 + beta*VehicleNumber + gama*SpeedFactor;
    }
    if ((CellTime >= 100) && (CellTime < 1001)) {
    return alpha*CellTime/100 + beta*VehicleNumber + gama*SpeedFactor;
    }
    if (CellTime > 1001){
    return alpha*CellTime/1000 + beta*VehicleNumber + gama*SpeedFactor;
    }*/
    return alpha*CellTime/10 + beta*VehicleNumber + gama*SpeedFactor;
}
int SERVitESApplLayer::calculateSpeedThereshold(int maxSpeed){

    int beta = 2 ;
    if (maxSpeed == 5)
        return beta * 1.3 ;
    if (maxSpeed==10)
        return beta *  2.4 ;
    if (maxSpeed==15)
           return beta *  3.34 ;
    if (maxSpeed==20)
           return beta *  4.58 ;
    if (maxSpeed==25)
           return beta *  5.8 ;
    if (maxSpeed==30)
           return beta * 6.67  ;
    if (maxSpeed==35)
           return beta * 7.9  ;
    if (maxSpeed==40)
           return beta *  8.89;
    // from journal

}

double SERVitESApplLayer::calculateTransmissionTimeFactor(){

    int CWmin = 15 ; //based IEEE 802.11p
    int CWmax = 1023 ; //based IEEE 802.11p
    double n ; // n = d * exp(-alpha*w) // w = Pnorm + Vnorm
    double teta = 0.9;
    double Pnorm;
    double Vnorm;
    double u;
    double speedTransmissionFactor;
    Coord myPosition = curPosition;
    Pnorm = (sqrt((myPosition.x-xmid)*(myPosition.x-xmid)+(myPosition.y-ymid)*(myPosition.y-ymid)))/(sqrt(xmid*xmid)+(ymid*ymid));

    EV << "get Current Position is : " << mobility->getCurrentPosition() << endl;
    EV << "get Position At is : " << mobility->getPositionAt(simTime()) << endl;
    EV << "curPosition is : " <<curPosition << endl;

    EV << "Pnorm is : " << Pnorm << endl;
    Vnorm = std::abs(mobility->getSpeed() - 2.24)/2.24 ;
    EV << "Vnorm is : " << Vnorm << endl;
    u = exp(-teta*(Pnorm + Vnorm)) ;
    EV << "chizi k mikham(u) : " << u << endl ;
    if (csim !=0){
    n = csim-u/u * exp(-teta*(Pnorm + Vnorm));
    }
    else {
        n = 1-u/u * exp(-teta*(Pnorm + Vnorm));
    }
    EV << "n is : " << n << endl;

    speedTransmissionFactor=hopthreshold*n*(CWmax-CWmin) + CWmin;

    if (speedTransmissionFactor <= 10){
        return speedTransmissionFactor;
    }
    if(speedTransmissionFactor > 10 && speedTransmissionFactor <=100){
        return speedTransmissionFactor/10;
    }
    if(speedTransmissionFactor > 100 && speedTransmissionFactor <=1000){
        return speedTransmissionFactor/100;
    }
    if(speedTransmissionFactor > 1000){
        return speedTransmissionFactor/1000;
    }

}

void SERVitESApplLayer::finish(){
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
    int GWDuration = 0;
    int numGWState = 0;
    int serviceAvailability = 0;
    int numserviceAvailability = 0;

    for (int i=0;i<counterSerial;i++){
        if (stateSerial[i] == CH){
            CHDuration++;
        }
        if (((stateSerial[i] == CH)&&(stateSerial[i+1] != CH))||((stateSerial[i] != CH)&&(stateSerial[i+1] == CH))){
            numCHState++;
        }
        if (stateSerial[i] == MV){
            MVDuration++;
        }
        if (((stateSerial[i] == MV)&&(stateSerial[i+1] != MV))||((stateSerial[i] != MV)&&(stateSerial[i+1] == MV))){
            numMVState++;
        }
    }
    for (int i=0;i<counterSerial;i++){
        if (GWSerial[i] == true){
            GWDuration++;
        }
        if ((GWSerial[i] != true)&&(GWSerial[i+1] == true)){
            numGWState++;
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
    //AvgCHDuration = double(CHDuration)/double(numCHState);
    AvgCHDuration = double(CHDuration);
    double AvgMVDuration = 0;
   // AvgMVDuration = double(MVDuration)/double(numMVState);
    AvgMVDuration = double(MVDuration);
    double AvgGWDuration = 0;
    AvgGWDuration = double(GWDuration)/double(numGWState);
    double AvgServiceAvailability = 0;
    AvgServiceAvailability = double(serviceAvailability)/double(numserviceAvailability);

    if (numCHState > 0){

        recordScalar("Avg CH Duration",AvgCHDuration);
        recordScalar("numCHchange",numCHState);
    }
    else {
        recordScalar("Avg CH Duration",0);
        recordScalar("numCHchange",0);
    }
    if (numMVState > 0){

        recordScalar("Avg MV Duration",AvgMVDuration);
        recordScalar("numMVchange",numMVState);
    }
    else {
        recordScalar("Avg MV Duration",0);
        recordScalar("numMVchange",0);
    }
    if (numGWState > 0){

      //  recordScalar("Avg GW Duration",AvgGWDuration);
    }
    else {
     //   recordScalar("Avg GW Duration",0);
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
    //recordScalar("Res Ready",resReadyNumber);
    //recordScalar("TEST VARIABLE" , testV);
    recordScalar("Service Searching TIME MIN ",serviceSearchingTimeMIN);
    recordScalar("Service Searching TIME MAX ",serviceSearchingTimeMAX);
    recordScalar("join cluster ",joinToCluster);
    recordScalar("overheadCluster", overheadCluster);
    recordScalar("overheadService", overheadService);
    recordScalar("overheadBeacon", overheadBeacon);
    recordScalar("member Number", memberNumber);
}
