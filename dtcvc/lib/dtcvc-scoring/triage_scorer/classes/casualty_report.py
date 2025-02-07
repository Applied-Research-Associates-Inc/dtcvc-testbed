from triage_scorer.constants.competitor_report_keys import *


class CasualtyReport:
    def __init__(
        self,
        drone_id=None,
        casualty_id=None,
        observation_start=None,
        assessment_time=None,
        observation_end=None,
        processing_end=None,
    ):
        self.drone_id = drone_id
        self.casualty_id = casualty_id
        self.observation_start = observation_start
        self.assessment_time = assessment_time
        self.observation_end = observation_end
        self.processing_end = processing_end
        self.vitals = Vitals()
        self.injuries = Injuries()
        self.location = Location()

    def to_dict(self):
        return {
            DRONE_ID: self.drone_id,
            CASUALTY_ID: self.casualty_id,
            OBSERVATION_START: self.observation_start,
            ASSESSMENT_TIME: self.assessment_time,
            OBSERVATION_END: self.observation_end,
            PROCESSING_END: self.processing_end,
            VITALS: {
                HEART_RATE: self.vitals.heart_rate,
                RESPIRATION_RATE: self.vitals.respiration_rate,
            },
            INJURIES: {
                SEVERE_HEMORRHAGE: self.injuries.severe_hemorrhage,
                RESPIRATORY_DISTRESS: self.injuries.respiratory_distress,
                TRAUMA: {
                    HEAD: self.injuries.trauma.head,
                    TORSO: self.injuries.trauma.torso,
                    UPPER_EXTREMITY: self.injuries.trauma.upper_extremity,
                    LOWER_EXTREMITY: self.injuries.trauma.lower_extremity,
                },
                ALERTNESS: {
                    OCULAR: self.injuries.alertness.ocular,
                    VERBAL: self.injuries.alertness.verbal,
                    MOTOR: self.injuries.alertness.motor,
                },
            },
            LOCATION: {X: self.location.x, Y: self.location.y, Z: self.location.z},
        }


class Vitals:
    def __init__(self, heart_rate=None, respiration_rate=None) -> None:
        self.heart_rate = heart_rate
        self.respiration_rate = respiration_rate


class Injuries:
    def __init__(self, severe_hemorrhage=None, respiratory_distress=None) -> None:
        self.severe_hemorrhage = severe_hemorrhage
        self.respiratory_distress = respiratory_distress
        self.trauma = Trauma()
        self.alertness = Alertness()


class Trauma:
    def __init__(self, head=None, torso=None, upper_extremity=None, lower_extremity=None) -> None:
        self.head = head
        self.torso = torso
        self.upper_extremity = upper_extremity
        self.lower_extremity = lower_extremity


class Alertness:
    def __init__(self, ocular=None, motor=None, verbal=None) -> None:
        self.ocular = ocular
        self.motor = motor
        self.verbal = verbal


class Location:
    def __init__(self, x=None, y=None, z=None) -> None:
        self.x = x
        self.y = y
        self.z = z
