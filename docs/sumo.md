# SUMO Network Files Overview

A SUMO network file describes the traffic-related part of a map: the roads and intersections that simulated vehicles use. At a coarse scale, a SUMO network is a directed graph:

- **Nodes** (called "junctions" in SUMO) represent intersections.
- **Edges** represent roads or streets (edges are unidirectional).

## Contents of a SUMO Network

A SUMO network contains:

- Every street (edge) as a collection of lanes, including position, shape, and speed limit of each lane.
- Traffic light logics referenced by junctions.
- Junctions, including right-of-way regulations.
- Connections between lanes at junctions (nodes).
- Optionally, districts and roundabout descriptions (depending on input formats and processing options).

> **Note:** Although SUMO network files are XML and human-readable, they are not meant to be edited by hand. Use SUMO XML description files with `NETCONVERT`, or convert existing maps from various formats. You can also generate abstract road maps with `NETGENERATE`, or modify `.net.xml` files using patch files or `NETEDIT`.

## Structure of SUMO Road Networks (XML)

SUMO road networks are encoded as XML files, typically grouped as follows:

1. Cartographic projection valid for the network.
2. Edges (internal edges first, then plain edges), each containing its lanes.
3. Traffic light logics.
4. Junctions (plain first, then internal), including right-of-way definitions.
5. Connections (plain first, then internal).
6. Optionally, roundabouts.

---

# OpenStreetMap Files for Traffic Simulation

An OpenStreetMap (OSM) file contains a map with features relevant to traffic simulation:

- Nodes and their connections define the position and form of streets and junctions.
- The type of street (key: `highway`) specifies its size and importance.
- Speed limits are usually implicit by law (based on `highway` value), but can be set explicitly with the `max_speed` key.
- The total number of lanes (for both directions) is usually implicit, but can be set with the `lanes` key.
- Traffic lights are described as nodes with `highway=traffic_signals`.
- One-way streets are marked with `oneway=yes`.

## Comparison: SUMO vs. OpenStreetMap File Formats

Compared to SUMO network files, OpenStreetMap files lack:

- The logic of traffic lights.
- The meaning of lanes and the connections between lanes at a junction.

---


