/*
 * txc6.cc
 *
 *  Created on: Mar 11, 2018
 *      Author: arihedy
 */
#include <stdio.h>
#include <string.h>
#include <omnetpp.h>

using namespace omnetpp;

//dar inja qasd darim nahve kar ba tabe scheduleAt() jahate ijad timer yad begirim

class Txc6 : public cSimpleModule
{
private:
    cMessage *event;  // pointer to the event object which we'll use for timing
    cMessage *tictocMsg;  // variable to remember the message until we send it back
public:
    Txc6();
    virtual ~Txc6();
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
};

Define_Module(Txc6);

Txc6::Txc6()
{
    // dar sorati k initialize seda zade nashod ham be moshkel nakhorim
    event=tictocMsg=nullptr;
}
Txc6::~Txc6()
{
    //be tore khodkar az tariq destructor msg haye tolid shode pak shavad
    cancelAndDelete(event);
    delete tictocMsg;
}
void Txc6::initialize()
{
    // Create the event object we'll use for timing -- just any ordinary message.
        event = new cMessage("event");

    //no tictoc msg yet
         tictocMsg=nullptr;

         if(strcmp("tic",getName())==0){
             //dar inja belafasele module tic payam ersal nemikonad balke ba yek takhir 5 s (saniye simulation)ersal mikonad (self-message)
             EV << "Scheduling first send to t=5.0s\n";
             tictocMsg= new cMessage ("tictocMsg");
             scheduleAt(5.0,event); // ba ijda yek msg be name event va ersal be khod zaman bandi khaste shode ijad mishavad.
         }
}
void Txc6::handleMessage(cMessage *msg)
{
     if(msg==event) // shart neveshte shode ra mitavan injori ham bayan kard: msg->isSelfMessage()
     { EV << "Wait period is over, sending back message\n";
        send(tictocMsg,"out");
        tictocMsg=nullptr;
 }
      else{
          //dar sorati k selfMessage nabashad payami ast k az module dgar miad va baad az yek saniye payam daryaft shode ersal misahvad
           EV << "Message arrived, starting to wait 1 sec...\n";
           tictocMsg=msg;
           scheduleAt(simTime()+1.0, event);
             }
   }
