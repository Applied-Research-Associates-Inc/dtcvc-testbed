"""Utility functions used to load the scenario, maps and ground truth data for the competition."""

import logging
import os
import re

import pandas as pd
import yaml

import triage_scorer.constants.ground_truth_keys as gtk


def load_scenario(filepath: str) -> dict | None:
    """Loads the scenario for the competition.

    Args:
        filepath (str): The filepath to the scenario

    Returns:
        dict | None: The scenario loaded as a dictionary or None
    """
    scenario: dict = None

    try:
        with open(filepath, "r") as file:
            scenario = yaml.safe_load(file)
    except FileNotFoundError:
        logging.error(f"Failed to open file | {filepath}")

    return scenario


def load_map(map_path: str) -> tuple[str, dict[str, tuple[float, float, float]]]:
    with open(map_path, "r") as file:
        map_name: str = file.readline().split(":")[0]
        # Move the cursor past the empty line
        file.readline()
        file_content: str = file.read()
        zone_entries = file_content.strip().split("\n--\n")

        gps_zone_locations: dict[str, tuple[float, float, float]] = {}
        for entry in zone_entries:
            lines = entry.split("\n")
            zone = lines[0].split(": ")[1]

            altitude = lines[1].split(": ")[1].split('"')[1]
            latitude = lines[2].split(": ")[1].split('"')[1]
            longitude = lines[3].split(": ")[1].split('"')[1]

            gps_zone_locations[zone] = (float(longitude), float(latitude), float(altitude))

        return map_name, gps_zone_locations


def load_maps(maps_dir: str) -> dict[str, dict[str, tuple[float, float, float]]]:
    """Loads the maps for the competition.

    Args:
        maps_dir (str): The directory path to the map files

    Returns:
        dict[str, dict[str, tuple[float, float, float]]]: The maps with map names as keys and the GPS zone locations as values
    """
    maps: dict[str, dict[str, tuple[float, float, float]]] = {}

    maps_dir_contents: list[str] = sorted(os.listdir(maps_dir))
    for filename in maps_dir_contents:
        filepath: str = os.path.join(maps_dir, filename)

        try:
            map_name, gps_zone_locations = load_map(filepath)
            maps[map_name] = gps_zone_locations
        except FileNotFoundError:
            logging.error(f"Failed to open file | {filepath}")

    return maps


def load_ground_truth(ground_truth_dir: str) -> dict[str, pd.DataFrame]:
    """Loads the ground truth data for the competition.

    Args:
        ground_truth_dir (str): The directory path to the ground truth CSVs

    Returns:
        dict[str, pd.DataFrame]: Mapping of casualty types to ground truth data as a pandas.DataFrame
    """
    ground_truth: dict[str, pd.DataFrame] = {}
    ground_truth_dir_contents: list[str] = sorted(os.listdir(ground_truth_dir))

    # The ground truth columns that will be referenced when scoring
    scored_cols: list[str] = [
        gtk.HEART_RATE,
        gtk.RESPIRATION_RATE,
        gtk.HA_SEVERE_HEMORRHAGE,
        gtk.HA_RESPIRATORY_DISTRESS,
        gtk.HA_HEAD_TRAUMA,
        gtk.HA_TORSO_TRAUMA,
        gtk.HA_UPPER_EXTREMITY_TRAUMA,
        gtk.HA_LOWER_EXTREMITY_TRAUMA,
        gtk.HA_OCULAR_ALERTNESS,
        gtk.HA_VERBAL_ALERTNESS,
        gtk.HA_MOTOR_ALERTNESS,
    ]

    for filename in ground_truth_dir_contents:
        csv_file: str = os.path.join(ground_truth_dir, filename)
        # Get the casualty type from the CSV filename
        pattern = r".*VTCScenario0(\d*)_.*\.csv"
        match = re.search(pattern, csv_file)
        type = str(match.group(1)).strip()
        casualty_type: str = "BP_Casualty_" + type

        df = pd.read_csv(csv_file)[["Time", *scored_cols]]
        ground_truth[casualty_type] = df
        df._gt_name = filename

    return ground_truth

def get_simulation_runtime(filepath: str) -> int | None:
    """Gets the simulation runtime from the scenario.yaml.

    Returns:
        int | None: The scenario runtime defined in the scenario.yaml or None
    """
    scenario: dict | None = load_scenario(filepath)
    runtime: int | None = None

    if scenario is not None:
        runtime = int(scenario["runtime"])

    return runtime
