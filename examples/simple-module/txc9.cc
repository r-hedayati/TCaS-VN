/*
 * txc9.cc
 *
 *  Created on: Mar 11, 2018
 *      Author: arihedy
 */
#include <stdio.h>
#include <string.h>
#include <omnetpp.h>

using namespace omnetpp;

class Tic9 : public cSimpleModule
{
private:

    simtime_t timeout; //timeout
    cMessage *timeoutEvent; // holds pointer to the timeout self-message
    int seq ; // message sequence number
    cMessage *message ; // message that has to be re-sent on timeout

protected:

    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void sendCopyOf (cMessage *msg);
    virtual cMessage* generateNewMessage();


public:

    Tic9();
    virtual ~Tic9();
};

Define_Module(Tic9);

Tic9::Tic9()
{
    message=timeoutEvent=nullptr;
}

Tic9::~Tic9()
{
    cancelAndDelete(timeoutEvent);
    delete message;
}

void Tic9::initialize()
{
    seq=0;
    timeout=1.0;
    timeoutEvent=new cMessage ("timeoutEvent");

    //Generate and send initial message.
    EV <<"Sending initial message\n";
    message = generateNewMessage();
    sendCopyOf(message);
    scheduleAt(simTime()+timeout,timeoutEvent);
    }

cMessage* Tic9::generateNewMessage()
{
// Generate a message with a different name every time.
    char msgname[20];
    sprintf(msgname,"tic-%d",++seq);
    cMessage *msg=new cMessage("msgname");
    return msg;
}

void Tic9::sendCopyOf(cMessage *msg)
{
    cMessage *copy = (cMessage* )msg->dup(); // dulicate msg then send it
    send(copy,"out");
}

void Tic9::handleMessage(cMessage *msg)
{
    if(msg==timeoutEvent)
    {
        EV<<"Timeout expired, resending message and restarting timer\n";

        sendCopyOf(message);
        scheduleAt(simTime()+timeout,timeoutEvent);
    }
    else
    {
        EV<<"Received: " << msg->getName() << std::endl;
        delete msg;

        EV<<"timer cancelled/n";
        cancelEvent(timeoutEvent);
        delete message;

        //send another one
        message=generateNewMessage();
        sendCopyOf(message);
        scheduleAt(simTime()+timeout, timeoutEvent);
    }
}

class Toc9 : public cSimpleModule
{
protected:
    virtual void handleMessage(cMessage *msg) override;
};

Define_Module(Toc9);

void Toc9::handleMessage(cMessage *msg)
{
    if(uniform(0,1)<0.1)
    {
        EV << "\"Losing\" message " << msg << endl;
            bubble("message lost");
            delete msg;
    }
    else
    {
        EV << msg << " received, sending back an acknowledgement.\n";
            delete msg;
            send(new cMessage("ack"), "out");
    }
}
