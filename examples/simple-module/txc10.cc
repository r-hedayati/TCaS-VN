/*
 * txc10.cc
 *
 *  Created on: Mar 12, 2018
 *      Author: arihedy
 */
/**
     * Let's make it more interesting by using several (n) `tic' modules,
     * and connecting every module to every other. For now, let's keep it
     * simple what they do: module 0 generates a message, and the others
     * keep tossing it around in random directions until it arrives at
     * module 3.
     */

#include <stdio.h>
#include <string.h>
#include <omnetpp.h>

using namespace omnetpp;


class Txc10 :public cSimpleModule
{
protected:
        virtual void forwardMessage(cMessage *msg);
        virtual void initialize() override;
        virtual void handleMessage(cMessage *msg) override;
};

Define_Module(Txc10);

void Txc10::initialize()
{
    // shore konnade ersal gate shomare 0 mibashad
    if(getIndex()==0) // az anja ke dar .ned be sorate vector tarif kardim inja jahate yaftane kodam gate az "getIndex" estefade mikonim
    {
        // Boot the process scheduling the initial message as a self-message.
            char msgname[20];
            sprintf(msgname, "tic-%d", getIndex());
            cMessage *msg = new cMessage(msgname);
            scheduleAt(0.0, msg);
    }
}
void Txc10::handleMessage(cMessage *msg)
{

            if (getIndex() == 3) {
                // Message arrived.
                EV << "Message " << msg << " arrived.\n";
                delete msg;
            }
            else {
                // We need to forward the message.
                forwardMessage(msg);
            }
    }
void Txc10::forwardMessage(cMessage* msg)
{
    int n= gateSize("out"); //tedade gate ha ro midhahad
    int k =  intuniform(0,n-1); //random beyne shomare gate ha entekhab mikonad.
    send(msg,"out",k);

    // dar sorati k az gate inout estefade konim :
    // int n= gateSize("gate");
    // int k=intuniform(0,n-1);
    //    EV << "Forwarding message " << msg << " on gate[" << k << "]\n";
    // $o and $i suffix is used to identify the input/output part of a two way gate
    //send(msg, "gate$o", k);

}
