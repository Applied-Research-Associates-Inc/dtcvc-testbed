from collections import OrderedDict
from triage_scorer.constants import competitor_report_keys as crk
from triage_scorer import report_validator
from triage_scorer.constants.competitor_report_keys import *


def format_report(report):
    """Format a report for scoring"""
    return merge_reports(report, generate_new_report())


def merge_reports(current_report: OrderedDict, previous_report: OrderedDict):
    """Verifies the validity of received casualty report fields"""

    merged_report = __resolve_dicts(current_report, previous_report)

    # Merge vitals section
    if crk.VITALS in current_report:
        merged_report[crk.VITALS] = merge_report_section(
            section_key=crk.VITALS,
            current_report=current_report,
            previous_report=previous_report,
            fields=[crk.HEART_RATE, crk.RESPIRATION_RATE],
        )

    # Merge injuries section
    if crk.INJURIES in current_report:
        merged_report[crk.INJURIES] = merge_report_section(
            section_key=crk.INJURIES,
            current_report=current_report,
            previous_report=previous_report,
            fields=[crk.SEVERE_HEMORRHAGE, crk.RESPIRATORY_DISTRESS],
        )

        # Merge trauma section
        if crk.TRAUMA in current_report[crk.INJURIES]:
            merged_report[crk.INJURIES][crk.TRAUMA] = merge_report_section(
                section_key=crk.TRAUMA,
                current_report=current_report[crk.INJURIES],
                previous_report=previous_report[crk.INJURIES],
                fields=[crk.HEAD, crk.TORSO, crk.UPPER_EXTREMITY, crk.LOWER_EXTREMITY],
            )

        # Merge alertnes section
        if crk.ALERTNESS in current_report[crk.INJURIES]:
            merged_report[crk.INJURIES][crk.ALERTNESS] = merge_report_section(
                section_key=crk.ALERTNESS,
                current_report=current_report[crk.INJURIES],
                previous_report=previous_report[crk.INJURIES],
                fields=[crk.OCULAR, crk.VERBAL, crk.MOTOR],
            )
    # Merge location section
    if crk.LOCATION in current_report:
        merged_report[crk.LOCATION] = merge_report_section(
            section_key=crk.LOCATION,
            current_report=current_report,
            previous_report=previous_report,
            fields=[crk.LON, crk.LAT, crk.ALT],
        )

    return merged_report


def merge_report_section(section_key, current_report, previous_report, fields):
    """Checks report section for format violations and merges two reports if no violations"""
    report_validator.check_report_section(current_report[section_key], parent_key=section_key, fields=fields)

    return __resolve_dicts(current_report[section_key], previous_report[section_key])


def __resolve_dicts(current_dict: dict, previous_dict: dict) -> dict:
    """Merges two dictionaries"""
    merged_dict = {}

    for key in previous_dict:
        prev_field = previous_dict[key]
        if not isinstance(prev_field, dict) and key in current_dict and current_dict[key] != None:
            merged_dict[key] = current_dict[key]
        else:
            merged_dict[key] = prev_field

    return merged_dict


def generate_new_report():
    """Generate blank report"""
    return {
        DRONE_ID: None,
        CASUALTY_ID: None,
        OBSERVATION_START: None,
        ASSESSMENT_TIME: None,
        OBSERVATION_END: None,
        PROCESSING_END: None,
        VITALS: {
            HEART_RATE: None,
            RESPIRATION_RATE: None,
        },
        INJURIES: {
            SEVERE_HEMORRHAGE: None,
            RESPIRATORY_DISTRESS: None,
            TRAUMA: {
                HEAD: None,
                TORSO: None,
                UPPER_EXTREMITY: None,
                LOWER_EXTREMITY: None,
            },
            ALERTNESS: {
                OCULAR: None,
                VERBAL: None,
                MOTOR: None,
            },
        },
        LOCATION: {
            LON: None,
            LAT: None,
            ALT: None,
        },
    }
