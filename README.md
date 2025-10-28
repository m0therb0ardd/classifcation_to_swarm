## Coachbot Live System

A simplified directory for running the live classification → swarm choreography system directly on the Coachbot network at Northwestern.

### Overview 
This folder contains the minimal working setup required to run my Laban-inspired gesture classification and swarm choreography system live on the Coachbot platform.

The project detects live human movement (via a RealSense camera), classifies gestures using a Random Forest model, and sends corresponding swarm mode commands to a group of Coachbots in a call-and-response style dance. 

This repo pulls from and builds on several of my other repositories:

- Classifier training: m0therb0ardd/laban_classifier: Contains the dataset collection, feature extraction, and model training pipeline for the gesture classifier.

- Choreography simulation: m0therb0ardd/coachbot_swarm_dance_simulation: Used for designing and testing different movement behaviors and swarm formations before deploying to the real robots.

- Coachbot lab system (private):: Coachbot-Swarm/m0therb0ardd: My private testing repo within the Northwestern Coachbot lab’s system, where I developed and submitted user code for robot deployment before creating this standalone working directory with direct access to the coachbot system.

### How it Works
1. Continuous Classification (10_continuous_classification.py)

    - Reads frames from the Intel RealSense camera

    - Uses MediaPipe Pose to extract 3D keypoints

    - Computes motion features and classifies gestures via random_forest_model.pkl

    - When a gesture surpasses a confidence threshold, updates swarm_config.json with:

        {
        "mode": "encircling",
        "source": {"label": "stillness", "type": "live_classify"},
        "timestamp": 1730144000.0
        }

2. Swarm Control (apply_from_json.py) 

    - Monitors swarm_config.json for changes

    - When a new mode is detected, runs the matching usr_code_<mode>.py behavior on selected robots via cctl commands

    - Example flow: cctl on 3 4 5; cctl update usr_code_encircling.py; cctl start

3. Behavior Scripts (usr_code_*.py)

    - Define distinct movement “modes” (e.g., glitch, float, encircling, directional_left/right)

    - Correspond to gesture classifications based on Laban movement qualities.