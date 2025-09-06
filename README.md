# TCaS-VN

**TCaS-VN: A Novel V2V Communication Scheme to Improve Traffic Control and Safety in Vehicular Cloud Networks**

This repository contains the source code and resources for the paper:[TCaS-VN: A novel V2V communication scheme to improve traffic control and safety in vehicular cloud networks](https://onlinelibrary.wiley.com/doi/10.1002/dac.5635)

## Overview

TCaS-VN is a vehicular cloud network simulation platform focused on V2V communication for traffic control and safety. The codebase includes simulation scenarios, algorithms, sample maps, and documentation to reproduce the results and experiments described in the paper.

## Repository Structure

-   `src/` — Main source code (C++ files, algorithms, etc.)
-   `maps/` — SUMO/OpenStreetMap files for simulation scenarios
    -   `amirkabir/` — Amirkabir map and related files
    -   `test-maps/` — Test maps for simulation
-   `examples/` — Example projects and scenarios
-   `docs/` — Documentation, algorithm descriptions, and guides
    -   `omnetpp/` — OMNeT++ guides and manuals
-   `images/` — Diagrams and figures for documentation
-   `dependencies/` — Required packages and dependencies (SUMO, Veins, etc.)

## Getting Started

1.  **Install Dependencies**
    
    -   SUMO (Simulation of Urban Mobility)
    -   OMNeT++ (Discrete event simulator)
    -   Veins (Vehicular network simulation framework)
    -   See `dependencies/` for installation files and guides.
2.  **Build the Project**
    
    -   Navigate to `src/` and compile the C++ source files using your preferred build system.
3.  **Run Simulations**
    
    -   Use the provided configuration files in `maps/` and `examples/` to run simulations.
    -   Refer to `docs/` for step-by-step instructions.

## Documentation

-   Algorithm details: `docs/algorithm-idea.pdf`
-   Simulation setup: `docs/sumo.md`, `docs/omnetpp/SimulationManual.pdf`
-   Pseudocode: `docs/pseudo-code(TCaS-VN).md`
-   Helpful resources: `docs/helpful-websites.md`

## Citation

If you use this codebase in your research, please cite the original paper:

```
@article{hedayati2025tcasvn,  title={TCaS-VN: A novel V2V communication scheme to improve traffic control and safety in vehicular cloud networks},  author={Hedayati, R. and others},  journal={International Journal of Communication Systems},  year={2025},  doi={10.1002/dac.5635}}
```

## License

This project is licensed under the [MIT License](LICENSE).