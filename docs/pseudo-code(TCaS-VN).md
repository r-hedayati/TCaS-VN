## Initial states

```python
status = ND
resourceState = FREE
```

## Node Discovery Phase
```python
if (is_ND) then
    send(HelloMSG)
end if
```

## Upon receiving HelloMSG
```python
if (receive_HelloMSG) then
    calculateCSF

    if (is_ND) then
        if (My_CSF > CSF_Packet) then
            send(CH_CandidateMSG)
        else
            send(HelloResponseMSG)
        end if
    end if

    if (state == is_Cluster) then
        send(HelloResponseMSG)
    end if
end if
```

## Upon receiving HelloResponseMSG
```python
if (receive_HelloResponseMSG) then
    calculateCSF

    if (the cells are the same) then
        if (time <= TTF_TransmissionTime) then
            if (is_ND) then
                if (My_CSF > CSF_Packet) then
                    send(CH_CandidateMSG)
                else
                    state <- MV
                    wait for a cluster to join
                end if
            end if

            if (is_Cluster) then
                if (My_CSF <= CSF_Packet) then
                    state <- MV
                    sendJoinMSG
                else
                    wait for another HelloResponseMSG to receive_HelloMSG
                end if
            end if
        else
            if (receiveMSG_During_waiting) then
                // do nothing
            else
                state <- CH
                send(CH_CandidateMSG)
            end if
        end if
    end if
end if

```

## Upon receiving CH_CandidateMSG

```python
if (receive CH_CandidateMSG) then
    calculateCSF
    if (the cells are the same) then
        if (My_CSF > CSF_Packet) then
            state <- CH
            send(CH_SelectionMSG)
        else
            state <- MV
            try to join a cluster
        end if
    end if
end if

```

## Upon receiving CH_SelectionMSG

```python
if (receive CH_SelectionMSG) then
    if (the cells are the same) then
        if (state is MV or ND) then
            if (receive one CH_SelectionMSG) then
                if (hop < hopTH) then
                    if (My_CSF <= CSF_Packet) then
                        state <- MV
                        sendJoinMSG
                    else
                        state <- MV
                        wait for another cluster to join
                    end if
                end if
            end if
            if (receive more than one CH_SelectionMSG) then
                state <- MV
                isGateway <- true
                if (not joining to any cluster) then
                    if (My_CSF <= CSF_Packet) then
                        sendJoinMSG
                    else
                        wait for another cluster to join
                    end if
                end if
            end if
        end if
    end if
end if
```

## Upon receiving JoinMSG

```python
if (receive JoinMSG) then
    if (the cells are the same) then
        if (state is MV and ID_CH is the same with ID_CH_Packet) then
            if (My_CSF > CSF_Packet) then
                accept node & store the information
            else
                CH is changed
                send(warningMSG)
                send(reorganizingMSG)
            end if
        end if
    end if
end if
```

## Cluster maintenance
check the parameters for maintenance of cluster

## Resource request phase
```python
if (needs some resources) then
    send(reqMSG)
end if
```
```python
if (receive reqMSG) then
    if (time < waiting_time_for_reqResponseMSG) then
        if (state is CH or isGateway is true) then
            find provider for requester
            store the information of provider
            if (requester can't find any resources from its neighbour's members) then
                send(reqMSG) to specific provider
            end if
            if (CH or GW can't find any provider in their cluster) then
                CH send(reqMSG) to another cluster
            end if
        end if
        if (state is MV) then
            if (resources are FREE) then
                resState <- BUSY
                send(reqResponseMSG)
            end if
        end if
    else
        requester can't find any resources from its members
        requester send(reqMSG) to its CH or GW
        send(reqMSG)
    end if
end if
```

```python
if (receive reqResponseMSG) then
    if (myID and ProviderID are the same) then
        resState <- BUSY
        send(ResReadyMSG)
    end if
    if (state is CH and the ReqMSG) then
        send(reqResponseMSG) to requester
    end if
end if
```

```python
if (receive ResReadyMSG) then
    if (myID and RequesterID are the same) then
        resState <- Allocate
        start using Resources (duration time of service is calculated)
    end if
end if
```














