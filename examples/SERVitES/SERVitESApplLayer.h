/*
 * SERVitESApplLayer.h
 *
 *  Created on: Apr 3, 2018
 *      Author: Montajab Ghanem
 */

#ifndef SERVITESAPPLLAYER_H_
#define SERVITESAPPLLAYER_H_


#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;

/**
 * Cloud VANET using 11p
 */
class SERVitESApplLayer : public BaseWaveApplLayer {
    public:
        virtual void initialize(int stage);
        virtual void finish();
        virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);
        enum MSG_TYPES
                {
                    HELLO_MSG, RESPONSE_MSG, CANDIDATE_MSG, CONTROL_MSG, BEACON_MSG, JOIN_MSG,GATEWAY_MAG,
                    QUERY_MSG, RESPONSE_QUERY_MSG
                };
        enum TIMERS{
                    HELLO_MSG_TIMER, WAITING_RESPONSE_TIMER, BEACON_TIMER, WAITING_AFTER_CANDIDATE_MSG,
                    RESET_TIMER
        };
        enum STATE {
                    CH, MV, GW, ND
        };
    protected:
        class Car {
            public:
                int id;
            public:
                Car (int i=-1):
                    id(i){
                };
            };
        typedef std::list<Car> cCarList;
        cCarList carList;
        cCarList ResponsedCarList;
        class Controller {
            public:
                int id;
                simtime_t tcell;
                simtime_t tsim;
                int csim;
                int hopCount;
                bool operator<(Controller const &other)  { return tsim > other.tsim; }
            public:
                Controller (int i=-1,simtime_t tc = 0,simtime_t ts =0,int cs = 0, int hc=0 ):
                    id(i), tcell(tc), tsim(ts),csim (cs), hopCount(hc){
                };
            };
        typedef std::list<Controller> cControllerList;
        cControllerList controllerList;
        class ConnectedV {
            public:
                int id;
                simtime_t tcell;
                simtime_t tsim;
                int csim;
                int hopCount;
                double res;
                bool operator<(Controller const &other)  { return tsim > other.tsim; }
            public:
                ConnectedV (int i=-1,simtime_t tc = 0,simtime_t ts =0,int cs = 0, int hc=0, double re = 1 ):
                    id(i), tcell(tc), tsim(ts),csim (cs), hopCount(hc), res(re){
                };
            };
        typedef std::list<ConnectedV> cConnectedVList;
        cConnectedVList connectedVList;
        class MemberV {
            public:
                int id;
                simtime_t tcell;
                simtime_t tsim;
                int csim;
                std::string traje;
                int hopCount;
                bool operator<(MemberV const &other)  { return tsim > other.tsim; }
            public:
                MemberV (int i=-1,simtime_t tc = 0,simtime_t ts =0,int cs = 0,std::string tr ="", int hc=0 ):
                    id(i), tcell(tc), tsim(ts),csim (cs),traje(tr), hopCount(hc){
                };
            };
        typedef std::list<MemberV> cMemberVList;
        cMemberVList memberVList;
        class Requester {
            public:
                int id;
                int id_requester;
                simtime_t tcell;
                simtime_t tsim;
                int csim;
                std::string traje;
                int hopCount;
                bool operator<(MemberV const &other)  { return tsim > other.tsim; }
            public:
                Requester (int i=-1,int idr = 0,simtime_t tc = 0,simtime_t ts =0,
                        int cs = 0,std::string tr ="", int hc=0 ):
                    id(i),id_requester(idr), tcell(tc), tsim(ts),csim (cs),traje(tr), hopCount(hc){
                };
            };
        typedef std::list<Requester> cRequesterList;
        cRequesterList requesterList;
        TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;
        AnnotationManager* annotations;
        simtime_t lastDroveAt;
        bool sentMessage;
        bool isParking;
        bool sendWhileParking;
        static const simsignalwrap_t parkingStateChangedSignal;
        // New variables
        cMessage* helloMsgTimer;
        cMessage* waitingResponseTimer;
        cMessage* beaconTimer;
        cMessage* waitingAfterCandidateMsg;
        cMessage* resetTimer;
        double res;
        int state;//the state indicates whether the sender is CH, MV, GW, or ND;
        int stateSerial[1000];
        int counterSerial;
        int ID_Controller;
        double xmid = 8500;
        double ymid = 10900;
        int csim;
        int cellnumber;
        bool receiveMsgDuringWaiting;
        bool waitingTimeExpireed;
        bool helloMsgSent;
        bool candidateMsgSent;
        bool controllerMsgSent;
        bool beaconMsgSent;
        bool sendQueryMessage;
        bool receivedControlMsgAfterCandidate;
        simtime_t tsim;
        simtime_t tcell;
        simtime_t transmissionTime;
        simtime_t beaconInterval;

        cOutVector CHDuration;
        double maxSpeed;
        int ttl;
        int hopthreshold;
        int kindGateway; // 1 for internal GW, 2 for bridge GW
        int ID_Gateway;
        int ID_reuester;
protected:
        virtual void onBeacon(WaveShortMessage* wsm);
        virtual void onData(WaveShortMessage* wsm);
        void relayPacket(WaveShortMessage* wsm);
        virtual void handleSelfMsg(cMessage* msg) ;
        virtual void handleLowerMsg(cMessage* msg);
        void sendMessage(std::string blockedRoadId);
        virtual void handlePositionUpdate(cObject* obj);
        virtual void handleParkingUpdate(cObject* obj);
        virtual void sendWSM(WaveShortMessage* wsm);
        //Defined function in Cloud-VANET
        int getCellNumber();
        Coord GPSLocationEstimation();
        simtime_t tsimCalculate(std::string road1,std::string road2);
        double distance(double x1,double y1,double x2,double y2,double z1,double z2);
        void sendCandidateMsg();
        void maintenance();
        void sendControllerMsg();
        void sendBeaconMsg();
        void sendGatewayMsg();
        void useResource();
        void sendJoinMsg();
        void sendQueryMsg();
        void sendQueryResponseMsg();
        simtime_t calculateTCell();
        int getCellNumber(Coord position);
};




#endif /* SERVITESAPPLLAYER_H_ */
