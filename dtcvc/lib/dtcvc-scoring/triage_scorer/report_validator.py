from collections import OrderedDict
from triage_scorer.constants import competitor_report_keys as crk
from triage_scorer.constants import enumerations as enm
from triage_scorer import report_resolver
from triage_scorer import exceptions
from itertools import chain


def check_required(report: dict):
    """Checks if a competitor report has the required fields set"""
    required_fields = [
        crk.DRONE_ID,
        crk.CASUALTY_ID,
        crk.OBSERVATION_START,
        crk.OBSERVATION_END,
        crk.ASSESSMENT_TIME,
        crk.PROCESSING_END,
        crk.LOCATION,
    ]

    for field in required_fields:
        if field not in report or report[field] == None:
            raise exceptions.RequiredFieldNotSetException(field)
        else:
            __verify_field(field, report[field])


def check_report_section(report_section: dict, parent_key: str, fields: list):
    """Checks a section of a casualty report"""
    if isinstance(report_section, dict):
        for field in fields:
            if field in report_section:
                __verify_field(field, report_section[field])
    else:
        raise exceptions.FieldInvalidException(parent_key, report_section, "dictionary")


def __verify_field(field: str, value):
    """Check an individual report field"""
    if value != None:
        try:
            match field:
                case crk.DRONE_ID | crk.CASUALTY_ID:
                    if int(value) < 0:
                        raise exceptions.FieldInvalidException(field, value)
                case crk.OBSERVATION_START | crk.OBSERVATION_END | crk.ASSESSMENT_TIME | crk.PROCESSING_END:
                    if float(value) <= 0.0:
                        raise exceptions.FieldInvalidException(field, value)
                case crk.HEART_RATE | crk.RESPIRATION_RATE:
                    if float(value) < 0.0:
                        raise exceptions.FieldInvalidException(field, value)
                case crk.SEVERE_HEMORRHAGE | crk.RESPIRATORY_DISTRESS:
                    if value not in enm.GENERAL_INJURY_STATES:
                        raise exceptions.FieldInvalidException(field, value)
                case crk.HEAD | crk.TORSO:
                    if value not in enm.HEAD_TORSO_TRAUMA_STATES:
                        raise exceptions.FieldInvalidException(field, value)
                case crk.UPPER_EXTREMITY | crk.LOWER_EXTREMITY:
                    if value not in enm.EXTREMITY_TRAUMA_STATES:
                        raise exceptions.FieldInvalidException(field, value)
                case crk.OCULAR:
                    if value not in enm.OCULAR_STATES:
                        raise exceptions.FieldInvalidException(field, value)
                case crk.VERBAL:
                    if value not in enm.VERBAL_STATES:
                        raise exceptions.FieldInvalidException(field, value)
                case crk.MOTOR:
                    if value not in enm.MOTOR_STATES:
                        raise exceptions.FieldInvalidException(field, value)
                # TODO: Remove after integrating x/y/z checking
                case crk.LOCATION:
                    if not isinstance(value, dict):
                        raise exceptions.FieldInvalidException(field, value)
                case crk.LON | crk.LAT | crk.ALT:
                    # No bounds, check if location is a valid float
                    float(value)
                case _:
                    raise exceptions.UnknownFieldException(field)
        except:
            raise exceptions.FieldInvalidException(field, value)
