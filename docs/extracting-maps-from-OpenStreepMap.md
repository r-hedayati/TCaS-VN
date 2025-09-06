# Extracting Maps from OpenStreetMap for SUMO

> **Note:** Ensure your `SUMO_HOME` environment variable is correctly set.

## Step-by-Step Guide

### 1. Download the Map
- Visit [openstreetmap.org](https://www.openstreetmap.org) and export your area of interest.
- Save the file as `map.osm`.

### 2. Open SUMO Command Line
- Run `start-command-line.bat` in the `sumo/bin` directory.

### 3. Set Destination Folder
- Place your `map.osm` file in your working directory.

### 4. Convert OSM to SUMO Network
```bash
netconvert --osm-files map.osm -o <output_name>.net.xml
```

### 5. Create Type Map
- Generate `typemap.xml` using [SUMO's OSM Import Guide](http://www.sumo.dlr.de/wiki/Networks/Import/OpenStreetMap).

### 6. Convert Polygons
```bash
polyconvert --net-file <output_name>.net.xml --osm-files map.osm --type-file typemap.xml -o <output_name>.poly.xml
```

### 7. Generate Random Trips
```bash
python C:/sumo/tools/randomTrips.py -n <output_name>.net.xml -e 100 -l
python C:/sumo/tools/randomTrips.py -n <output_name>.net.xml -r <routes_name>.rou.xml -e 100 -l
```
- `-e 100` sets the number of vehicles to 100.

### 8. Configure Simulation
- Copy and edit a config file (e.g., `test.sumo.cfg`).
- Update references to your new files and add any additional files as needed.

### 9. Run SUMO GUI
```bash
sumo-gui <output_name>.sumo.cfg
```

---

This guide helps you extract, convert, and simulate OpenStreetMap data in SUMO efficiently.