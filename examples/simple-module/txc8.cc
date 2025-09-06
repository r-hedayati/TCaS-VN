//
	// This file is part of an OMNeT++/OMNEST simulation example.
	//
	// Copyright (C) 2003-2015 Andras Varga
	//
	// This file is distributed WITHOUT ANY WARRANTY. See the file
	// `license' for details on this and other legal matters.
	//

	#include <stdio.h>
	#include <string.h>
	#include <omnetpp.h>

	using namespace omnetpp;

	//dar in mesal be elat raftarhaye motefaveti ke har module darad  dar kelas e jodagane tarif mishavad.
	//
	class Tic8 : public cSimpleModule
	{
	  private:
	    simtime_t timeout;  // timeout
	    cMessage *timeoutEvent;  // holds pointer to the timeout self-message

	  public:
	    Tic8();
	    virtual ~Tic8();

	  protected:
	    virtual void initialize() override;
	    virtual void handleMessage(cMessage *msg) override;
	};

	Define_Module(Tic8);

	Tic8::Tic8()
	{
	    timeoutEvent = nullptr;
	}

	Tic8::~Tic8()
	{
	    cancelAndDelete(timeoutEvent);
	}

	void Tic8::initialize()
	{
	    // Initialize variables.
	    timeout = 1.0;
	    timeoutEvent = new cMessage("timeoutEvent");

	    // Generate and send initial message.
	    EV << "Sending initial message\n";
	    cMessage *msg = new cMessage("tictocMsg");
	    send(msg, "out");
	    scheduleAt(simTime()+timeout, timeoutEvent);
	}
	//module tic pas az daryaft payam payam jadidi ersal mikonad va dar sorate tamam shodane timer dobare reset mikonad va mifrestad
	void Tic8::handleMessage(cMessage *msg)
	{
	    if (msg == timeoutEvent) {
	        // If we receive the timeout event, that means the packet hasn't
	        // arrived in time and we have to re-send it.
	        EV << "Timeout expired, resending message and restarting timer\n";
	        cMessage *newMsg = new cMessage("tictocMsg");
	        send(newMsg, "out");
	        scheduleAt(simTime()+timeout, timeoutEvent);
	    }
	    else {  // message arrived
	            // Acknowledgement received -- delete the received message and cancel
	            // the timeout event.
	        EV << "Timer cancelled.\n";
	        cancelEvent(timeoutEvent); //tavajoh shavad inja payam delet nashode balke faqat cancel shode
	        delete msg;

	        // Ready to send another one.
	        cMessage *newMsg = new cMessage("tictocMsg");
	        send(newMsg, "out");
	        scheduleAt(simTime()+timeout, timeoutEvent);
	    }
	}

	/**
	 * Sends back an acknowledgement -- or not.
	 */
	class Toc8 : public cSimpleModule
	{
	  protected:
	    virtual void handleMessage(cMessage *msg) override;
	};

	Define_Module(Toc8);

	//module toc aqazgar ersal nis pas initialize nadarad va serfan daryaft mikonad k dar sorati ke payam az dast raft be sorate geraphiki namayesh dade mishavad.
	// az tarafi baryae inke befahmim ack nayomade timeout inkaro baraye ma mikone vaqti payami daryaft nashe dar tic bade zamae timeout dobare ersal mishe
	void Toc8::handleMessage(cMessage *msg)
	{
	    if (uniform(0, 1) < 0.1) {
	        EV << "\"Losing\" message.\n";
	        bubble("message lost");  // making animation more informative...
	        delete msg;
	    }
	    else {
	        EV << "Sending back same message as acknowledgement.\n";
	        send(msg, "out");
	    }
	}
