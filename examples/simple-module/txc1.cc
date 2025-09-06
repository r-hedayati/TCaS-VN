/*
 * txc1.cc
 *
 *  Created on: Mar 10, 2018
 *      Author: arihedy
 */
#include <stdio.h>
#include <string.h>
#include <omnetpp.h>

using namespace omnetpp;

class Txc2 : public cSimpleModule //tamami module hayi k qarer sakhte she az module tarif shode dar omnetpp yani cSimpleModule estefade mikonand
{// 2 method majazi zir havmare tarif mishavad
protected:
    virtual void initialize() override; // algorithm haye marahele amade sazi
    virtual void handleMessage(cMessage *msg) override; //algorithm haye zamni k yek msg be module ma miresad ra dar bar migirad

private:
    int counter; // counter baraye inke pas az 10 bar payam nafreste dg.

};

Define_Module(Txc2);

void Txc2::initialize() {

    // moshakhas kardan inke che moduli ebteda aqaz konad ? tic ya toc ?

    counter =10;

    WATCH(counter); // method "WATCH" baraye moshahade taqirat moteqayer dar mohit Tkenv mibashad. After doing a few steps in the simulation, double-click either
    // `tic' or `toc', select the Contents tab in the dialog that pops up,
    // and you'll find "counter" in the list.

    EV<< "Sending initial message\n"; // jahate debug va check kardan dorosti code -- ya neveshtan e har chizi k mikhaym

    if (strcmp("tic",getName())==0) // strcmp injori kar mikone k 2 ta string migre moqayese mikone , dar inja getName ham check mikone k vorodi alan tic ya toc
    {
        cMessage *msg = new cMessage("tictocMsg"); // dar hafeze heep (stack) yek payam jadid ijad mikonim be esme "msg" ke ba name "tictocMsg" shenakhte mishe--tavajoh she k "tictocMsg" ekhtiarist!
        send(msg,"out"); //msg sakhte shode ra be port khoroji modulemon mifreste

    }

}

void Txc2::handleMessage(cMessage *msg){




    //dar inja mikhahim har gah be tedad counter ferestade shod qate shavad ersal.--> msg delet shavad
    // tavajoh shavad k sharte ersal msg bastegi be counter darad va hatman bayad dar dakhel shart neveshte shavad(motmaen nistam!)
    counter--;
    if (counter == 0) {
                // If counter is zero, delete message. If you run the model, you'll
                // find that the simulation will stop at this point with the message
                // "no more events".
                EV << getName() << "'s counter reached zero, deleting message\n";
                delete msg;
            }
            else {
                EV << getName() << "'s counter is " << counter << ", sending back message\n";
                send(msg, "out");
            }
        }



