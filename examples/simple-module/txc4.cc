/*
 * txc4.cc
 *
 *  Created on: Mar 11, 2018
 *      Author: arihedy
 */
#include <stdio.h>
#include <string.h>
#include <omnetpp.h>

using namespace omnetpp;

class Txc4 : public cSimpleModule
    {
      private:
        int counter;

      protected:
        virtual void initialize() override;
        virtual void handleMessage(cMessage *msg) override;
    };

Define_Module(Txc4);

void Txc4::initialize()
{
            counter = par("limit"); // dastor e "par()" jahate khandane parametr az file ned-- shabake sakhte shode

// baraye aqaz payam dgar mesle qabl nis meqdare moteqayer khande va agar dorost bod ersal mikonim.
            if (par("sendMsgOnInit").boolValue() == true) {
                EV << "Sending initial message\n";
                cMessage *msg = new cMessage("tictocMsg");
                send(msg, "out");
            }
        }
void Txc4::handleMessage(cMessage *msg)
    {
        counter--;
        if (counter == 0) {
            EV << getName() << "'s counter reached zero, deleting message\n";
            delete msg;
        }
        else {
            EV << getName() << "'s counter is " << counter << ", sending back message\n";
            send(msg, "out");
        }
    }
