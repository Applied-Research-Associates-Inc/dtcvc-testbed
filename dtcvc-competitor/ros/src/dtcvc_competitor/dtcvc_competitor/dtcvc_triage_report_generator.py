"""Will publish a single report and then spin forever"""

import json
import random
from std_msgs.msg import String

import triage_scorer.scoring as ts
import triage_scorer.constants.enumerations as enm
import triage_scorer.constants.competitor_report_keys as crk


class DtcvcTriageReportGenerator:
    """Utility class for generating random triage reports for debugging and testing"""

    REPORT_VALID = "valid"
    REPORT_RANDOM_VITALS = "random-vitals"
    REPORT_RANDOM_INJURIES = "random-injuries"
    REPORT_RANDOM_COMPLETE = "random-complete"

    REPORT_TYPES = [
        REPORT_VALID,
        REPORT_RANDOM_VITALS,
        REPORT_RANDOM_INJURIES,
        REPORT_RANDOM_COMPLETE,
    ]

    def __init__(self, drone_count=2, casualty_count=10):
        self.drone_count = drone_count
        self.casualty_count = casualty_count

    def publish_random(self, publisher):
        self.publish(
            publisher,
            random.sample(DtcvcTriageReportGenerator.REPORT_TYPES, 1)[0],
            random.randint(1, self.drone_count),
            random.randint(1, self.casualty_count),
        )

    def publish(self, publisher, report_type: str, drone_id: int, casualty_id: int):
        report = None
        match report_type:
            case DtcvcTriageReportGenerator.REPORT_VALID:
                report = self.generate_valid_victim_report(drone_id=drone_id, casualty_id=casualty_id)
            case DtcvcTriageReportGenerator.REPORT_RANDOM_VITALS:
                report = self.generate_random_victim_vitals_only(drone_id=drone_id, casualty_id=casualty_id)
            case DtcvcTriageReportGenerator.REPORT_RANDOM_INJURIES:
                report = self.generate_random_victim_injuries_only(drone_id=drone_id, casualty_id=casualty_id)
            case DtcvcTriageReportGenerator.REPORT_RANDOM_COMPLETE:
                report = self.generate_random_victim_complete_report(drone_id=drone_id, casualty_id=casualty_id)
        if report is not None:
            publisher.publish(String(data=json.dumps(report)))

    def generate_random_victim_vitals_only(self, drone_id: int, casualty_id: int) -> dict:
        """Generates a victim report containing vitals data only.

        Args:
            drone_id (int): The ID of the drone
            casualty_id (int): The ID of the casualty

        Returns:
            dict: The victim report containing vitals data
        """
        self.ocular_states = list(enm.OCULAR_STATES)
        self.verbal_states = list(enm.VERBAL_STATES)
        self.motor_states = list(enm.MOTOR_STATES)
        self.general_injury_states = list(enm.GENERAL_INJURY_STATES)

        self.head_torso_trauma_states = list(enm.HEAD_TORSO_TRAUMA_STATES)
        self.extremity_trauma_states = list(enm.EXTREMITY_TRAUMA_STATES)

        temp_report = {}
        temp_report[crk.OBSERVATION_START] = random.uniform(0, 100)
        assessment_time = random.uniform(temp_report[crk.OBSERVATION_START], 100)
        temp_report[crk.OBSERVATION_END] = assessment_time + random.uniform(0, 10)
        temp_report[crk.ASSESSMENT_TIME] = assessment_time
        temp_report[crk.CASUALTY_ID] = casualty_id
        temp_report[crk.DRONE_ID] = drone_id

        temp_report[crk.LOCATION] = {}
        temp_report[crk.LOCATION][crk.LON] = random.uniform(-180, 180)
        temp_report[crk.LOCATION][crk.LAT] = random.uniform(-90, 90)
        temp_report[crk.LOCATION][crk.ALT] = random.uniform(-10000, 10000)

        temp_report[crk.VITALS] = {}
        temp_report[crk.VITALS][crk.HEART_RATE] = random.uniform(50, 120)
        temp_report[crk.VITALS][crk.RESPIRATION_RATE] = random.uniform(0, 50)

        return temp_report

    def generate_random_victim_injuries_only(self, drone_id: int, casualty_id: int) -> dict:
        """Generates a victim report containing injuries data only.

        Args:
            drone_id (int): The ID of the drone
            casualty_id (int): The ID of the casualty

        Returns:
            dict: The victim report containing injuries data
        """
        self.ocular_states = list(enm.OCULAR_STATES)
        self.verbal_states = list(enm.VERBAL_STATES)
        self.motor_states = list(enm.MOTOR_STATES)
        self.general_injury_states = list(enm.GENERAL_INJURY_STATES)

        self.head_torso_trauma_states = list(enm.HEAD_TORSO_TRAUMA_STATES)
        self.extremity_trauma_states = list(enm.EXTREMITY_TRAUMA_STATES)

        temp_report = {}
        temp_report[crk.OBSERVATION_START] = random.uniform(0, 100)
        assessment_time = random.uniform(temp_report[crk.OBSERVATION_START], 100)
        temp_report[crk.OBSERVATION_END] = assessment_time + random.uniform(0, 10)
        temp_report[crk.ASSESSMENT_TIME] = assessment_time
        temp_report[crk.CASUALTY_ID] = casualty_id
        temp_report[crk.DRONE_ID] = drone_id

        temp_report[crk.LOCATION] = {}
        temp_report[crk.LOCATION][crk.LON] = random.uniform(-180, 180)
        temp_report[crk.LOCATION][crk.LAT] = random.uniform(-90, 90)
        temp_report[crk.LOCATION][crk.ALT] = random.uniform(-10000, 10000)

        temp_report[crk.INJURIES] = {}
        temp_report[crk.INJURIES][crk.SEVERE_HEMORRHAGE] = random.choice([True, False])
        temp_report[crk.INJURIES][crk.RESPIRATORY_DISTRESS] = random.choice([True, False])

        temp_report[crk.INJURIES][crk.TRAUMA] = {}
        temp_report[crk.INJURIES][crk.TRAUMA][crk.HEAD] = random.sample(self.head_torso_trauma_states, 1)[0]
        temp_report[crk.INJURIES][crk.TRAUMA][crk.TORSO] = random.sample(self.head_torso_trauma_states, 1)[0]
        temp_report[crk.INJURIES][crk.TRAUMA][crk.UPPER_EXTREMITY] = random.sample(self.extremity_trauma_states, 1)[0]
        temp_report[crk.INJURIES][crk.TRAUMA][crk.LOWER_EXTREMITY] = random.sample(self.extremity_trauma_states, 1)[0]

        temp_report[crk.INJURIES][crk.ALERTNESS] = {}
        temp_report[crk.INJURIES][crk.ALERTNESS][crk.OCULAR] = random.sample(self.ocular_states, 1)[0]
        temp_report[crk.INJURIES][crk.ALERTNESS][crk.VERBAL] = random.sample(self.verbal_states, 1)[0]
        temp_report[crk.INJURIES][crk.ALERTNESS][crk.MOTOR] = random.sample(self.motor_states, 1)[0]

        return temp_report

    def generate_random_victim_complete_report(self, drone_id: int, casualty_id: int) -> dict:
        """Generates a complete victim report containing all possible data fields.

        Args:
            drone_id (int): The ID of the drone
            casualty_id (int): The ID of the casualty

        Returns:
            dict: The complete victim report
        """
        self.ocular_states = list(enm.OCULAR_STATES)
        self.verbal_states = list(enm.VERBAL_STATES)
        self.motor_states = list(enm.MOTOR_STATES)
        self.general_injury_states = list(enm.GENERAL_INJURY_STATES)

        self.head_torso_trauma_states = list(enm.HEAD_TORSO_TRAUMA_STATES)
        self.extremity_trauma_states = list(enm.EXTREMITY_TRAUMA_STATES)

        temp_report = {}
        temp_report[crk.OBSERVATION_START] = random.uniform(0, 100)
        assessment_time = random.uniform(temp_report[crk.OBSERVATION_START], 100)
        temp_report[crk.OBSERVATION_END] = assessment_time + random.uniform(0, 10)
        temp_report[crk.ASSESSMENT_TIME] = assessment_time
        temp_report[crk.CASUALTY_ID] = casualty_id
        temp_report[crk.DRONE_ID] = drone_id

        temp_report[crk.LOCATION] = {}
        temp_report[crk.LOCATION][crk.LON] = random.uniform(-180, 180)
        temp_report[crk.LOCATION][crk.LAT] = random.uniform(-90, 90)
        temp_report[crk.LOCATION][crk.ALT] = random.uniform(-10000, 10000)

        temp_report[crk.VITALS] = {}
        temp_report[crk.VITALS][crk.HEART_RATE] = random.uniform(50, 120)
        temp_report[crk.VITALS][crk.RESPIRATION_RATE] = random.uniform(0, 50)

        temp_report[crk.INJURIES] = {}
        temp_report[crk.INJURIES][crk.SEVERE_HEMORRHAGE] = random.choice([True, False])
        temp_report[crk.INJURIES][crk.RESPIRATORY_DISTRESS] = random.choice([True, False])

        temp_report[crk.INJURIES][crk.TRAUMA] = {}
        temp_report[crk.INJURIES][crk.TRAUMA][crk.HEAD] = random.sample(self.head_torso_trauma_states, 1)[0]
        temp_report[crk.INJURIES][crk.TRAUMA][crk.TORSO] = random.sample(self.head_torso_trauma_states, 1)[0]
        temp_report[crk.INJURIES][crk.TRAUMA][crk.UPPER_EXTREMITY] = random.sample(self.extremity_trauma_states, 1)[0]
        temp_report[crk.INJURIES][crk.TRAUMA][crk.LOWER_EXTREMITY] = random.sample(self.extremity_trauma_states, 1)[0]

        temp_report[crk.INJURIES][crk.ALERTNESS] = {}
        temp_report[crk.INJURIES][crk.ALERTNESS][crk.OCULAR] = random.sample(self.ocular_states, 1)[0]
        temp_report[crk.INJURIES][crk.ALERTNESS][crk.VERBAL] = random.sample(self.verbal_states, 1)[0]
        temp_report[crk.INJURIES][crk.ALERTNESS][crk.MOTOR] = random.sample(self.motor_states, 1)[0]

        return temp_report

    def generate_valid_victim_report(self, drone_id: int, casualty_id: int) -> dict:
        """Generates a hard-coded report that should always successfully validate against the current version of the schema.

        Args:
            drone_id (int): The ID of the drone
            casualty_id (int): The ID of the casualty

        Returns:
            dict: The victim report with hard-coded data
        """
        report = {
            "observation_start": 100.0,
            "observation_end": 115.0,
            "assessment_time": 115.0,
            "casualty_id": casualty_id,
            "drone_id": drone_id,
            "location": {"lon": -117.123456, "lat": 35.987654, "alt": 1000},
            "vitals": {"heart_rate": 80, "respiration_rate": 50},
            "injuries": {
                "severe_hemorrhage": True,
                "respiratory_distress": True,
                "trauma": {"head": "wound"},
                "alertness": {
                    "ocular": "open",
                    "verbal": "abnormal",
                    "motor": "absent",
                },
            },
        }

        return report
