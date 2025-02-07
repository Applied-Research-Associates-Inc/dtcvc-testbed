"""Will publish a single casualty report and then spin forever"""

import json
import random
from std_msgs.msg import String

import triage_scorer.constants.casualty_report_keys as crk


_SEVERE_HEMORRHAGE_STATES = { 0, 1 }
_RESPIRATORY_DISTRESS_STATES = { 0, 1 }
_TRAUMA_HEAD_STATES = { 0, 1, 2 }
_TRAUMA_TORSO_STATES = { 0, 1, 2 }
_TRAUMA_LOWER_EXT_STATES = { 0, 1, 2, 3 }
_TRAUMA_UPPER_EXT_STATES = { 0, 1, 2, 3 }
_ALERTNESS_OCULAR_STATES = { 0, 1, 2 }
_ALERTNESS_VERBAL_STATES = { 0, 1, 2, 3 }
_ALERTNESS_MOTOR_STATES = { 0, 1, 2, 3 }


class DtcvcTriageReportGenerator:
    """Utility class for generating random casualty reports for debugging and testing"""

    REPORT_NON_RANDOM_COMPLETE = "non-random-complete"
    REPORT_RANDOM_COMPLETE = "random-complete"

    REPORT_TYPES = [
        REPORT_NON_RANDOM_COMPLETE,
        REPORT_RANDOM_COMPLETE,
    ]

    def __init__(self, casualty_count=10, team_count=3, system_count=3):
        self.casualty_count = casualty_count
        self.team_count = team_count
        self.system_count = system_count

    def publish_random(self, publisher):
        self.publish(
            publisher,
            random.sample(DtcvcTriageReportGenerator.REPORT_TYPES, 1)[0],
            random.randint(1, self.casualty_count),
            random.randint(1, self.team_count),
            random.randint(1, self.system_count)
        )

    def publish(self, publisher, report_type: str, casualty_id: int, team_id: int, system_id: int):
        report = None
        match report_type:
            case DtcvcTriageReportGenerator.REPORT_NON_RANDOM_COMPLETE:
                report = self.generate_non_random_casualty_report(casualty_id=casualty_id, team=f"team-{team_id}", system=f"system-{system_id}")
            case DtcvcTriageReportGenerator.REPORT_RANDOM_COMPLETE:
                report = self.generate_random_casualty_report(casualty_id=casualty_id, team=f"team-{team_id}", system=f"system-{system_id}")
        if report is not None:
            publisher.publish(String(data=json.dumps(report)))

    def generate_random_casualty_report(
            self, 
            casualty_id: int, 
            team: str, 
            system: str) -> dict:
        """Generates a casualty report with all health assessment fields defined.

        Args:
            casualty_id (int): The ID of the casualty
            team (str): The team ID
            system (str): The drone ID

        Returns:
            dict: Casualty report with random health assessment data
        """
        self.severe_hemorrhage_states = list(_SEVERE_HEMORRHAGE_STATES)
        self.respiratory_distress_states = list(_RESPIRATORY_DISTRESS_STATES)
        self.trauma_head_states = list(_TRAUMA_HEAD_STATES)
        self.trauma_torso_states = list(_TRAUMA_TORSO_STATES)
        self.trauma_lower_ext_states = list(_TRAUMA_LOWER_EXT_STATES)
        self.trauma_upper_ext_states = list(_TRAUMA_UPPER_EXT_STATES)
        self.alertness_ocular_states = list(_ALERTNESS_OCULAR_STATES)
        self.alertness_verbal_states = list(_ALERTNESS_VERBAL_STATES)
        self.alertness_motor_states = list(_ALERTNESS_MOTOR_STATES)

        temp_report = {}
        temp_report[crk.CASUALTY_ID] = casualty_id
        temp_report[crk.TEAM] = team 
        temp_report[crk.SYSTEM] = system 
        temp_report[crk.LOCATION] = {}
        temp_report[crk.LOCATION][crk.LATITUDE] = random.uniform(-90, 90)
        temp_report[crk.LOCATION][crk.LONGITUDE] = random.uniform(-180, 180)
        temp_report[crk.LOCATION][crk.TIME_AGO] = random.uniform(15.0, 45.0)
        temp_report[crk.SEVERE_HEMORRHAGE] = random.sample(self.severe_hemorrhage_states, 1)[0]
        temp_report[crk.RESPIRATORY_DISTRESS] = random.sample(self.respiratory_distress_states, 1)[0]
        temp_report[crk.HEART_RATE] = {}
        temp_report[crk.HEART_RATE][crk.HA_VALUE] = random.uniform(50.0, 120.0)
        temp_report[crk.HEART_RATE][crk.TIME_AGO] = random.uniform(50.0, 210.0)
        temp_report[crk.RESPIRATION_RATE] = {}
        temp_report[crk.RESPIRATION_RATE][crk.HA_VALUE] = random.uniform(0.0, 50.0)
        temp_report[crk.RESPIRATION_RATE][crk.TIME_AGO] = random.uniform(50.0, 210.0)
        temp_report[crk.CORE_TEMP] = {}
        temp_report[crk.CORE_TEMP][crk.HA_VALUE] = random.uniform(20.0, 40.0)
        temp_report[crk.CORE_TEMP][crk.TIME_AGO] = random.uniform(50.0, 210.0)
        temp_report[crk.TRAUMA_HEAD] = random.sample(self.trauma_head_states, 1)[0]
        temp_report[crk.TRAUMA_TORSO] = random.sample(self.trauma_torso_states, 1)[0]
        temp_report[crk.TRAUMA_LOWER_EXTREMITY] = random.sample(self.trauma_lower_ext_states, 1)[0]
        temp_report[crk.TRAUMA_UPPER_EXTREMITY] = random.sample(self.trauma_upper_ext_states, 1)[0]
        temp_report[crk.ALERTNESS_OCULAR] = {}
        temp_report[crk.ALERTNESS_OCULAR][crk.HA_VALUE] = random.sample(self.alertness_ocular_states, 1)[0]
        temp_report[crk.ALERTNESS_OCULAR][crk.TIME_AGO] = random.uniform(50.0, 210.0)
        temp_report[crk.ALERTNESS_VERBAL] = {}
        temp_report[crk.ALERTNESS_VERBAL][crk.HA_VALUE] = random.sample(self.alertness_verbal_states, 1)[0]
        temp_report[crk.ALERTNESS_VERBAL][crk.TIME_AGO] = random.uniform(50.0, 210.0)
        temp_report[crk.ALERTNESS_MOTOR] = {}
        temp_report[crk.ALERTNESS_MOTOR][crk.HA_VALUE] = random.sample(self.alertness_motor_states, 1)[0]
        temp_report[crk.ALERTNESS_MOTOR][crk.TIME_AGO] = random.uniform(50.0, 210.0)

        return temp_report

    def generate_non_random_casualty_report(
            self, 
            casualty_id: int, 
            team: str, 
            system: str) -> dict:
        """Generates a casualty report that should always successfully validate against the current version of the schema.

        Args:
            casualty_id (int): The ID of the casualty
            team (str): The team ID
            system (str): The drone ID

        Returns:
            dict: Casualty report with non-random health assessment data
        """
        report = {
            "casualty_id": casualty_id,
            "team": team,
            "system": system,
            "location": {
                "latitude": 48.926884,
                "longitude": 8.110344,
                "time_ago": 15.0
            },
            "severe_hemorrhage": 0,
            "respiratory_distress": 1,
            "hr": {
                "value": 75.0,
                "time_ago": 30.0
            },
            "rr": {
                "value": 13.0,
                "time_ago": 45.0
            },
            "temp": {
                "value": 36.5,
                "time_ago": 60.0
            },
            "trauma_head": 0,
            "trauma_torso": 0,
            "trauma_lower_ext": 0,
            "trauma_upper_ext": 0,
            "alertness_ocular": {
                "value": 0,
                "time_ago": 75.0
            },
            "alertness_verbal": {
                "value": 2,
                "time_ago": 75.0
            },
            "alertness_motor": {
                "value": 2,
                "time_ago": 90.0
            }
        }

        return report
