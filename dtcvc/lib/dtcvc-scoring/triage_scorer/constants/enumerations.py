"""Defines all enumerations used in Phase 1 and Phase 2."""
from enum import Enum 


# Casualty Report Enumerations

class SevereHemorrhage(Enum):
    ABSENT = 0
    PRESENT = 1


class RespiratoryDistress(Enum):
    ABSENT = 0
    PRESENT = 1


class TraumaHead(Enum):
    NORMAL = 0
    WOUND = 1
    NOT_TESTABLE = 2


class TraumaTorso(Enum): 
    NORMAL = 0
    WOUND = 1
    NOT_TESTABLE = 2


class TraumaLowerExt(Enum):
    NORMAL = 0
    WOUND = 1
    AMPUTATION = 2
    NOT_TESTABLE = 3


class TraumaUpperExt(Enum):
    NORMAL = 0
    WOUND = 1
    AMPUTATION = 2
    NOT_TESTABLE = 3


class AlertnessOcular(Enum):
    OPEN = 0 
    CLOSED = 1
    NOT_TESTABLE = 2


class AlertnessVerbal(Enum): 
    NORMAL = 0
    ABNORMAL = 1
    ABSENT = 2
    NOT_TESTABLE = 3


class AlertnessMotor(Enum):
    NORMAL = 0
    ABNORMAL = 1
    ABSENT = 2
    NOT_TESTABLE = 3

# Error Enumerations

class ReportRejection(Enum):
    INVALID_REPORT = 0
    """Error indicating report did not pass validation"""

    NO_NEARBY_GT = 1
    """Error indicating a report was not in the proximity of a ground truth"""

    EXCESS_CASUALTY = 2
    """Error indicating a report had a new casualty id despite the max number of casualties already being tracked"""

    EXCESS_UPDATE = 3
    """Error indicating an update already exists for an initial report"""

    UPDATE_GT_MISMATCH = 4
    """Error indicating the update is not in the proximity of the same ground truth as the initial report"""


# Tolerance Enumerations

class Tolerance(Enum):
    LOCATION = 2.0
    """Defines the location tolerance in which reported locations must be within in order to receive points"""

    HEART_RATE = 5.0
    """Defines the heart rate tolerance in which assessments must be under in order to receive points"""

    RESPIRATION_RATE = 3.0
    """Defines the respiration rate tolerance in which assessments must be under in order to receive points"""

    CORE_TEMPERATURE = 2.0
    """Defines the core temperature tolerance in which assessments must be under in order to receive points"""
