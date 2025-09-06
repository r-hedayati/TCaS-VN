/*
 * txc13.cc
 *
 *  Created on: Mar 12, 2018
 *      Author: arihedy
 */

#include <stdio.h>
    #include <string.h>
    #include <omnetpp.h>
#include "tictoc13_m.h"

    using namespace omnetpp;

    class Txc13 : public cSimpleModule
    {
      protected:
        virtual TicTocMsg13 *generateMessage();
        virtual void forwardMessage(TicTocMsg13 *msg);
        virtual void initialize() override;
        virtual void handleMessage(cMessage *msg) override;
        virtual void refreshDisplay() const override;

        // The finish() function is called by OMNeT++ at the end of the simulation:
            virtual void finish() override;

      private:
        long numSent;
        long numReceived;
        cLongHistogram hopCountStats;
        cOutVector hopCountVector;

    };

    Define_Module(Txc13);

    void Txc13::initialize()
    {
        numSent=0;
        numReceived=0;
        WATCH(numSent);
        WATCH(numReceived);



        hopCountStats.setName("hopCountStats");
        hopCountStats.setRangeAutoUpper(0, 10, 1.5);
        hopCountVector.setName("HopCount");


        //module 0 start
        if(getIndex()==0);
        TicTocMsg13 *msg=generateMessage(); // dar inja dgar be jaye cMessage *msg --> az msg k khodeman sakhtim estefade mikonim.
        scheduleAt(0.0,msg);

    }
    TicTocMsg13 *Txc13::generateMessage()
        { // az anja k dar msg sakhte shode field hayi gozashtim inja bayad meqdar dahi konim.
            // Produce source and destination addresses.

        int src = getIndex();// shomare module k toshim migire
        int n = getVectorSize(); //tedad gate ha ro migire
        int dest = intuniform(0,n-2); // maqsad ra random moshakhas mikonad.

        if(dest>=src)
            dest++;

        char msgname[20];
        sprintf(msgname, "tic-%d-to%d", src, dest);

        //create msg object and set src & dest field.

        TicTocMsg13 *msg = new TicTocMsg13(msgname);
        msg->setSource(src);
        msg->setDestination(dest);
        return msg;
        }
    void Txc13::handleMessage(cMessage *msg)
    {

        TicTocMsg13 *ttmsg = check_and_cast<TicTocMsg13 *>(msg); // bayad format msg daryafti be msg khodemon taqir bedim.
        if(ttmsg->getDestination()== getIndex()){
            //msg arrived
            EV << "Message" << ttmsg << "arrived after" << ttmsg->getHopCount() << "hops.\n";
            bubble("Arrived, starting over");
            int hopcount = ttmsg->getHopCount();

            // update statistics.
                numReceived++;
                hopCountVector.record(hopcount);
                hopCountStats.collect(hopcount);


            delete ttmsg;


            //Generate another one
            EV << "Generating another message: ";
                TicTocMsg13 *newmsg = generateMessage();
                EV << newmsg << endl;
                forwardMessage(newmsg);
                numSent++;
        }
        else
        {
            forwardMessage(ttmsg);
        }
    }
void Txc13::forwardMessage(TicTocMsg13 *msg){

        msg->setHopCount(msg->getHopCount()+1);

        int n = gateSize("gate");
        int k = intuniform(0, n-1);

        EV << "Forwarding message " << msg << " on gate[" << k << "]\n";
        send(msg, "gate$o", k);

}
void Txc13::refreshDisplay() const
    {
        char buf[40];
        sprintf(buf, "rcvd: %ld sent: %ld", numReceived, numSent);
        getDisplayString().setTagArg("t", 0, buf);
    }

void Txc13::finish()
    {
        // This function is called by OMNeT++ at the end of the simulation.
        EV << "Sent:     " << numSent << endl;
        EV << "Received: " << numReceived << endl;
        EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
        EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
        EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
        EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;

        recordScalar("#sent", numSent);
        recordScalar("#received", numReceived);

        hopCountStats.recordAs("hop count");
    }
