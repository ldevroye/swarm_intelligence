# swarm intelligence project

small ARGoS swarm robotics project for a tunnelling task

## goal

maximize the score

score = #robots in black zone - #obstacles in black zone

this is a local reactive controller for foot-bot robots running in ARGoS

## project layout

- `controller.lua` : main robot controller
- `blind.argos` : main experiment config used by the project
- `script.argos` : alternate config
- `build.sh` : build helper for the ARGoS plugin
- `src/` : native C++ plugin sources
- `doc/` : project notes and technical references
- `exercices/` : course examples and reference controllers

## run

1. build the plugin

```bash
./build.sh
```

2. run the experiment headless

```bash
srcargos && argos3 -c blind.argos
```

or, if the project is configured to use the provided wrapper:

```bash
srcargos && argos3 -c blind.argos 2>out.txt
```

## important notes

- the controller is loaded by `blind.argos` via the `script` param
- the simulation uses a black target area and a light source
- robot behavior is based on local sensing and local communication
- logs are written per robot under the `logs/` folder

## data to inspect

- `output.txt` : simulation output from loop functions
- `logs/` : per-robot log files
- `tunnel.log` : produced by the headless run workflow when available

## behavior overview

- orient toward light and establish swarm spacing
- form a group in the tunnel phase
- move toward the black target zone
- maintain low-level obstacle avoidance and black-floor tracking
