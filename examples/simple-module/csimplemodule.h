 //==========================================================================
     //   CSIMPLEMODULE.H  -  header for
     //                     OMNeT++/OMNEST
     //            Discrete System Simulation in C++
     //
    //==========================================================================
     
     /*--------------------------------------------------------------*
       Copyright (C) 1992-2017 Andras Varga
      Copyright (C) 2006-2017 OpenSim Ltd.
    
      This file is distributed WITHOUT ANY WARRANTY. See the file
      `license' for details on this and other legal matters.
    *--------------------------------------------------------------*/
    
    #ifndef __OMNETPP_CSIMPLEMODULE_H
    #define __OMNETPP_CSIMPLEMODULE_H
   
    #include "cmodule.h"
    
    namespace omnetpp {
    
    class cQueue;
    class cCoroutine;
    
    class SIM_API cSimpleModule : public cModule //implies noncopyable
    {
        friend class cModule;
       friend class cSimulation;
    
      private:
       enum {
            FL_USESACTIVITY        = 1 << 12, // uses activity() or handleMessage()
            FL_ISTERMINATED        = 1 << 13, // for both activity and handleMessage modules
            FL_STACKALREADYUNWOUND = 1 << 14, // only for activity modules
      };
   
        cMessage *timeoutMessage;   // msg used in wait() and receive() with timeout
        cCoroutine *coroutine;
    
        static bool stackCleanupRequested; // 'true' value asks activity() to throw a cStackCleanupException
        static cSimpleModule *afterCleanupTransferTo; // transfer back to this module (or to main)
    
      private:
        // internal use
        static void activate(void *p);
    
      protected:
        // internal use
        virtual void arrived(cMessage *msg, cGate *ongate, simtime_t t) override;
    
      protected:
   
       virtual void activity();
   
       virtual void handleMessage(cMessage *msg);
  
    public:
       cSimpleModule(unsigned stacksize = 0);
   
       cSimpleModule(const char *dummy1, cModule *dummy2, unsigned stacksize);
  
       virtual ~cSimpleModule();
   
      virtual std::string str() const override;
  
       virtual void forEachChild(cVisitor *v) override;
   
   
       virtual void scheduleStart(simtime_t t) override;
  
       virtual void deleteModule() override;
   
   
       bool usesActivity() const  {return flags&FL_USESACTIVITY;}
   
      bool isTerminated() const {return flags&FL_ISTERMINATED;}
   
      virtual void snapshot(cObject *obj=nullptr, const char *label=nullptr);
   
   
      virtual void send(cMessage *msg, int gateid)  {return sendDelayed(msg, SIMTIME_ZERO, gateid);}
   
       virtual void send(cMessage *msg, const char *gatename, int gateindex=-1)  {return sendDelayed(msg, SIMTIME_ZERO, gatename, gateindex);}
   
       virtual void send(cMessage *msg, cGate *outputgate)  {return sendDelayed(msg, SIMTIME_ZERO, outputgate);}
  
       virtual void sendDelayed(cMessage *msg, simtime_t delay, int gateid);
   
       virtual void sendDelayed(cMessage *msg, simtime_t delay, const char *gatename, int gateindex=-1);
   
       virtual void sendDelayed(cMessage *msg, simtime_t delay, cGate *outputgate);
   
       virtual void sendDirect(cMessage *msg, cModule *mod, const char *inputGateName, int gateIndex=-1);
  
      virtual void sendDirect(cMessage *msg, cModule *mod, int inputGateId);
   
       virtual void sendDirect(cMessage *msg, cGate *inputGate);
   
       virtual void sendDirect(cMessage *msg, simtime_t propagationDelay, simtime_t duration, cModule *mod, const char *inputGateName, int gateIndex=-1);
   
       virtual void sendDirect(cMessage *msg, simtime_t propagationDelay, simtime_t duration, cModule *mod, int inputGateId);
   
      virtual void sendDirect(cMessage *msg, simtime_t propagationDelay, simtime_t duration, cGate *inputGate);
   
   
      virtual void scheduleAt(simtime_t t, cMessage *msg);
   
       virtual cMessage *cancelEvent(cMessage *msg);
   
      virtual void cancelAndDelete(cMessage *msg);
   
  
      virtual cMessage *receive();
   
       virtual cMessage *receive(simtime_t timeout);
   
   
       virtual void wait(simtime_t time);
  
       virtual void waitAndEnqueue(simtime_t time, cQueue *queue);
   
       virtual void endSimulation();
   
       virtual void halt();
   
       virtual void error(const char *format,...) const;
   
   
       virtual bool hasStackOverflow() const;
  
      virtual unsigned getStackSize() const;
   
       virtual unsigned getStackUsage() const;
   };
   
   }  // namespace omnetpp
   
   
   #endif
   