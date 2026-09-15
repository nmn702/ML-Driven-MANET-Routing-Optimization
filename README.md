# Adaptive Routing in MANETs using ML

This repository contains the implementation of the research paper: **"Adaptive Routing Strategies for Optimization in MANETs through Integration of Expanding Ring Search and Random Early Detection using Machine Learning"**.

## Overview

This project implements an adaptive routing strategy for Mobile Ad-hoc Networks (MANETs). To optimize routing overhead and congestion, it integrates:
- **Expanding Ring Search (ERS):** To intelligently discover routes without flooding the entire network.
- **Random Early Detection (RED):** For proactive queue management to prevent network congestion.
- **Machine Learning (KNN):** To dynamically control routing parameters based on current network state.

## Repository Structure

- `ml_controller.py`: The machine Learning controller script that processes network states and predicts routing parameters.
- `dsetgen1.cc` / `dset.cc`: C++ simulation scripts built for the **ns-3** network simulator to evaluate the proposed protocol.
- `knn_optimal_models.pkl`: Pre-trained K-Nearest Neighbors (KNN) model used for decision making.
- `manet_scaler.pkl`: The dataset scaler used alongside the KNN model.

## Requirements

- [ns-3](https://www.nsnam.org/) (Network Simulator 3) - Tested with version 3.48
- Python 3.x
- scikit-learn
- pandas
- numpy

## Usage

1. Copy the `dsetgen1.cc` and `dset.cc` files into the `scratch/` directory of your local `ns-3` installation.
2. Ensure the `.pkl` files and `ml_controller.py` are accessible by the Python environment running the simulations.
3. Run the simulation through the ns-3 build system:
   ```bash
   ./ns3 run scratch/dsetgen1
   ```

## License

This project is released under the MIT License.
