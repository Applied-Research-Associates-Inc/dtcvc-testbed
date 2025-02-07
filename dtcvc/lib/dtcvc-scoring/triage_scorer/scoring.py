import logging
from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional
import pathlib
import os

import json
import numpy as np
import pandas as pd
from geopy.distance import geodesic
from scipy.optimize import linear_sum_assignment

from triage_scorer.constants import casualty_report_keys as crk
from triage_scorer.constants import enumerations as enums
from triage_scorer.constants import final_report_keys as frk
from triage_scorer.constants import ground_truth_keys as gtk


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


@dataclass
class ScoredCasualty:
    casualty_id: int
    initial: dict
    update: dict
    hungarian_score: tuple[float, Optional[enums.ReportRejection]]
    hungarian_gt_loc: tuple[float, float, float]
    _score_array: list[tuple[float, Optional[enums.ReportRejection]]]


class ScoreTracker:
    def __init__(self, gts, limit=0):
        self._gts: list[tuple[tuple[float, float, float], pd.DataFrame]] = gts
        self._limit = limit
        self.tracked_casualties: dict[int, ScoredCasualty] = OrderedDict()
        self.__previous_hungarian_score_sum: float = 0.0

    def track_report(self, report: dict, report_time: float) -> Optional[enums.ReportRejection]:
        if report_time < 0.02:
            return enums.ReportRejection.INVALID_REPORT
        
        cid = report[crk.CASUALTY_ID]

        if cid not in self.tracked_casualties:
            if self._limit and not len(self.tracked_casualties) < self._limit:
                return enums.ReportRejection.EXCESS_CASUALTY
            self.tracked_casualties[cid] = ScoredCasualty(
                casualty_id=cid,
                initial=None,
                update=None,
                hungarian_score=None,
                hungarian_gt_loc=None,
                _score_array=None)
            self.tracked_casualties[cid].initial = report
        elif self.tracked_casualties[cid].update is not None:
            return enums.ReportRejection.EXCESS_UPDATE
        else:
            self.tracked_casualties[cid].update = report

        tc = self.tracked_casualties[cid]
        tc_loc = ((tc.initial[crk.LOCATION][crk.LATITUDE], tc.initial[crk.LOCATION][crk.LONGITUDE]))
        tc._score_array = [
            (
                score_report(report_time=report_time, loc_to_gt=gt, initial_report=tc.initial, report_update=tc.update)
                if (geodesic(tc_loc, (gt[0][1], gt[0][0])).m <= enums.Tolerance.LOCATION.value)
                else (0.0, enums.ReportRejection.NO_NEARBY_GT)
            )
            for gt in self._gts
        ]
        self.hungarian()

    def hungarian(self):
        keys = list(self.tracked_casualties.keys())     # used because keys/values of OrderedDict can't be indexed
        score_matrix = np.stack([
            self.tracked_casualties[cid]._score_array
            for cid in self.tracked_casualties
        ], axis=0)
        numeric_matrix = score_matrix[:, :, 0].astype(np.float64)   # Explicitly convert the slice to dtype=float64
        row_idxs, col_idxs = linear_sum_assignment(numeric_matrix, maximize=True)
        for row_idx, col_idx in zip(row_idxs, col_idxs):
            sc = self.tracked_casualties[keys[row_idx]]
            sc.hungarian_score = score_matrix[row_idx][col_idx]
            sc.hungarian_gt_loc = self._gts[col_idx][0]

        score_total: float = sum(map(lambda tc: tc.hungarian_score[0], self.tracked_casualties.values()))

        if score_total < self.__previous_hungarian_score_sum:
            logger.error("hungarian(): Hungarian computed lower score")

        self.__previous_hungarian_score_sum = score_total 


def score_report(
        report_time: float,
        loc_to_gt: tuple[tuple[float, float, float], pd.DataFrame], 
        initial_report: dict, 
        report_update: dict | None = None) -> tuple[float, enums.ReportRejection]:
    """Score initial report and report update if it exists.

    Args:
        loc_to_gt (tuple[float, float, float], pd.DataFrame): Mapping of location to ground truth casualty
        initial_report (dict): The initial casualty report
        report_update (dict): The update report
 
    Returns:
        The sum of the initial report score and report update score
    """
    score, err = 0, None
    score += sum(score_initial_report(report_time, loc_to_gt, initial_report))

    if report_update is not None:
        # Note: If we get here, then the initial report is already within the location tolerance
        report_update_loc = (report_update[crk.LOCATION][crk.LATITUDE], report_update[crk.LOCATION][crk.LONGITUDE])
        if geodesic(report_update_loc, (loc_to_gt[0][1], loc_to_gt[0][0])).m <= enums.Tolerance.LOCATION.value:
            score += score_report_update(loc_to_gt, initial_report, report_update) if report_update is not None else 0.0
        else:
            err = enums.ReportRejection.UPDATE_GT_MISMATCH

    return score, err


def score_initial_report(
    report_time: float, loc_to_gt: tuple[tuple[float, float, float], pd.DataFrame], initial_report: dict
) -> tuple[float, float]:
    """Score a validated casualty report against the nearest ground truth casualty.

    Args:
        report_time (float): Used to get the row containing the corresponding Golden Window value
        loc_to_gt (tuple[tuple[float, float, float], pd.DataFrame]): Mapping of casualty location to ground truth casualty data
        initial_report (dict): The initial casualty report

    Returns:
        tuple[float, float]: The score of the casualty report as a grouping of (scored_points, bonus_points); 25 possible scored points and 5 possible bonus points
    """
    scored_points: float = (
        5.0  # Points awarded for correct health assessments; starts at 5.0 because we already checked that it is within location tolerance
    )
    bonus_points: float = 0.0  # Points awarded for correctly assessing fields within the golden window
    _, gt = loc_to_gt  # Extract the ground truth data from the mapping

    # Time to determine Golden Window is based on report time
    in_gw: bool = get_time_proximal_row(gt, report_time)[gtk.GOLDEN_WINDOW]

    scored, bonus = score_severe_hemorrhage(gt=gt, report=initial_report, in_gw=in_gw)
    scored_points += scored
    bonus_points += bonus

    scored, bonus = score_respiratory_distress(gt=gt, report=initial_report, in_gw=in_gw)
    scored_points += scored
    bonus_points += bonus

    scored, bonus = score_vitals(gt=gt, report=initial_report, in_gw=in_gw)
    scored_points += scored
    bonus_points += bonus

    scored_points += score_core_temperature(gt=gt, report=initial_report)
    scored_points += score_trauma_head(gt=gt, report=initial_report)
    scored_points += score_trauma_torso(gt=gt, report=initial_report)
    scored_points += score_trauma_lower_ext(gt=gt, report=initial_report)
    scored_points += score_trauma_upper_ext(gt=gt, report=initial_report)
    scored_points += score_alertness(gt=gt, report=initial_report)

    # The minimum amount of points a report can have is 0
    if scored_points < 0.0:
        scored_points = 0.0

    return (scored_points, bonus_points)


def score_report_update(
    loc_to_gt: tuple[tuple[float, float, float], pd.DataFrame], initial_report: dict, report_update: dict | None
) -> float:
    """Scores the report update from the ReportTracker against the nearest ground truth casualty.

    Args:
        loc_to_gt (tuple[tuple[float, float, float], pd.DataFrame]): Mapping of casualty location to ground truth casualty data
        initial_report (dict): The initial casualty report
        report_update (dict | None): The report update

    Returns:
        float: The score of the report update
    """
    scored_points: float = (
        2.0  # Points awarded for correct health assessments; award 2.0 base points because reported location is within location tolerance
    )
    _, gt = loc_to_gt  # Extract the ground truth data from the mapping

    scored_points += score_heart_rate_update(gt=gt, report_update=report_update)
    scored_points += score_respiration_rate_update(gt=gt, report_update=report_update)
    scored_points += score_alertness_ocular_update(
        gt=gt, initial_report=initial_report, report_update=report_update
    )
    scored_points += score_alertness_verbal_update(
        gt=gt, initial_report=initial_report, report_update=report_update
    )
    scored_points += score_alertness_motor_update(gt=gt, initial_report=initial_report, report_update=report_update)

    return scored_points if scored_points > 0.0 else 0.0


def score_severe_hemorrhage(gt: pd.DataFrame, report: dict, in_gw: bool) -> tuple[float, float]:
    """Scores the severe hemorrhage assessment from the report against a ground truth casualty. Awards bonus points if the assessment occurred within the Golden Window.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report
        in_gw (bool): Determines whether bonus points will be awarded or not

    Returns:
        tuple[float, float]: Pairing of (scored_points, bonus_points)
    """
    if crk.SEVERE_HEMORRHAGE in report:
        report_value: int = report[crk.SEVERE_HEMORRHAGE]
        # NOTE: Using time_ago from location is a weird (temporary?) byproduct of copying the systems' track report format (at DARPA/APL's request)
        # Severe Hemorrhage and Respiratory Distress are constant in the systems track, so they aren't <value_time> fields in the report, but they probably
        # ought to be in our virtual track
        time_ago: float = report[crk.LOCATION][crk.TIME_AGO]
        assessment: str = get_severe_hemorrhage_assessment(report_value=report_value)

        row = get_time_proximal_row(gt, time_ago)

        if assessment == row[gtk.HA_SEVERE_HEMORRHAGE]:
            if in_gw:
                return (4.0, 2.0)
            else:
                return (4.0, 0.0)
        else:
            return (-4.0, 0.0)
    else:
        return (0.0, 0.0)


def score_respiratory_distress(gt: pd.DataFrame, report: dict, in_gw: bool) -> tuple[float, float]:
    """Scores the respiratory distress assessment from the report against a ground truth casualty. Awards bonus points if the assessment occurred within the Golden Window.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report
        in_gw (bool): Determines whether bonus points will be awarded or not

    Returns:
        tuple[float, float] Pairing of (scored_points, bonus_points)
    """
    if crk.RESPIRATORY_DISTRESS in report:
        report_value: int = report[crk.RESPIRATORY_DISTRESS]
        # NOTE: Using time_ago from location is a weird (temporary?) byproduct of copying the systems' track report format (at DARPA/APL's request)
        # Severe Hemorrhage and Respiratory Distress are constant in the systems track, so they aren't <value_time> fields in the report, but they probably
        # ought to be in our virtual track
        time_ago: float = report[crk.LOCATION][crk.TIME_AGO]
        assessment: str = get_respiratory_distress_assessment(report_value=report_value)

        row = get_time_proximal_row(gt, time_ago)

        if assessment == row[gtk.HA_RESPIRATORY_DISTRESS]:
            if in_gw:
                return (4.0, 2.0)
            else:
                return (4.0, 0.0)
        else:
            return (-4.0, 0.0)
    else:
        return (0.0, 0.0)


def score_vitals(gt: pd.DataFrame, report: dict, in_gw: bool) -> tuple[float, float]:
    """Calls the associated vitals scoring functions and determines whether additional and/or bonus points will be awarded.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report
        in_gw (bool): Determines whether bonus points will be awarded or not

    Returns:
        tuple[float, float]: Pairing of (scored_points, bonus_points)
    """
    points: float = 0.0
    points += score_heart_rate(gt=gt, report=report)
    points += score_respiration_rate(gt=gt, report=report)

    if points == 2.0:
        if in_gw:
            return (3.0, 1.0)
        else:
            return (3.0, 0.0)

    return (points, 0.0)


def score_heart_rate(gt: pd.DataFrame, report: dict) -> float:
    """Scores the heart rate assessment from the report against a ground truth casualty over the preceding 10 seconds.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.HEART_RATE in report:
        hr_value: float = report[crk.HEART_RATE][crk.HA_VALUE]
        time_ago: float = report[crk.HEART_RATE][crk.TIME_AGO]

        row = get_time_proximal_row(gt, time_ago)

        if abs(hr_value - row[gtk.HEART_RATE]) < enums.Tolerance.HEART_RATE.value:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_respiration_rate(gt: pd.DataFrame, report: dict):
    """Scores the respiration rate assessment from the report against a ground truth casualty over the preceding 60 seconds.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.RESPIRATION_RATE in report:
        rr_value: float = report[crk.RESPIRATION_RATE][crk.HA_VALUE]
        time_ago: float = report[crk.RESPIRATION_RATE][crk.TIME_AGO]
        time_window: float = 60.0

        series = get_time_proximal_row(
            gt, time_ago, time_window
        )  # series will be used to compute average rr over given time window
        avg_rr: float = compute_avg_rr(series, time_ago, time_window)

        if abs(rr_value - avg_rr) < enums.Tolerance.RESPIRATION_RATE.value:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_core_temperature(gt: pd.DataFrame, report: dict) -> float:
    """Scores the core temperature assessment from the report against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.CORE_TEMP in report:
        temp_value: float = report[crk.CORE_TEMP][crk.HA_VALUE]
        time_ago: float = report[crk.CORE_TEMP][crk.TIME_AGO]

        row = get_time_proximal_row(gt, time_ago)

        if abs(temp_value - row[gtk.CORE_TEMP]) < enums.Tolerance.CORE_TEMPERATURE.value:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_trauma_head(gt: pd.DataFrame, report: dict) -> float:
    """Scores the head trauma assessment from the report against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.TRAUMA_HEAD in report:
        report_value: int = report[crk.TRAUMA_HEAD]
        # NOTE: Using time_ago from location is a weird (temporary?) byproduct of copying the systems' track report format (at DARPA/APL's request)
        # Trauma is a constant value and will exist from start to end of a scenario, so the 'time_ago' provided with location will be sufficient
        time_ago: float = report[crk.LOCATION][crk.TIME_AGO]
        assessment: str = get_trauma_head_assessment(report_value=report_value)

        row = get_time_proximal_row(gt, time_ago)

        if assessment == row[gtk.HA_HEAD_TRAUMA]:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_trauma_torso(gt: pd.DataFrame, report: dict) -> float:
    """Scores the torso trauma assessment from the report against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.TRAUMA_TORSO in report:
        report_value: int = report[crk.TRAUMA_TORSO]
        # NOTE: Using time_ago from location is a weird (temporary?) byproduct of copying the systems' track report format (at DARPA/APL's request)
        # Trauma is a constant value and will exist from start to end of a scenario, so the 'time_ago' provided with location will be sufficient
        time_ago: float = report[crk.LOCATION][crk.TIME_AGO]
        assessment: str = get_trauma_torso_assessment(report_value=report_value)

        row = get_time_proximal_row(gt, time_ago)

        if assessment == row[gtk.HA_TORSO_TRAUMA]:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_trauma_lower_ext(gt: pd.DataFrame, report: dict) -> float:
    """Scores the lower extremity trauma assessment from the report against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.TRAUMA_LOWER_EXTREMITY in report:
        report_value: int = report[crk.TRAUMA_LOWER_EXTREMITY]
        # NOTE: Using time_ago from location is a weird (temporary?) byproduct of copying the systems' track report format (at DARPA/APL's request)
        # Trauma is a constant value and will exist from start to end of a scenario, so the 'time_ago' provided with location will be sufficient
        time_ago: float = report[crk.LOCATION][crk.TIME_AGO]
        assessment: str = get_trauma_lower_ext_assessment(report_value=report_value)

        row = get_time_proximal_row(gt, time_ago)

        if assessment == row[gtk.HA_LOWER_EXTREMITY_TRAUMA]:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_trauma_upper_ext(gt: pd.DataFrame, report: dict) -> float:
    """Scores the upper extremity trauma assessment from the report against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.TRAUMA_UPPER_EXTREMITY in report:
        report_value: int = report[crk.TRAUMA_UPPER_EXTREMITY]
        # NOTE: Using time_ago from location is a weird (temporary?) byproduct of copying the systems' track report format (at DARPA/APL's request)
        # Trauma is a constant value and will exist from start to end of a scenario, so the 'time_ago' provided with location will be sufficient
        time_ago: float = report[crk.LOCATION][crk.TIME_AGO]
        assessment: str = get_trauma_upper_ext_assessment(report_value=report_value)

        row = get_time_proximal_row(gt, time_ago)

        if assessment == row[gtk.HA_UPPER_EXTREMITY_TRAUMA]:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_alertness(gt: pd.DataFrame, report: dict) -> float:
    """Calls the associated alertness scoring functions and determines whether additional points will be awarded.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of all reported alertness health assessments
    """
    points: float = 0.0
    points += score_alertness_ocular(gt=gt, report=report)
    points += score_alertness_verbal(gt=gt, report=report)
    points += score_alertness_motor(gt=gt, report=report)

    if points == 3.0:
        return 4.0

    return points


def score_alertness_ocular(gt: pd.DataFrame, report: dict) -> float:
    """Scores the ocular alertness assessment from the report against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.ALERTNESS_OCULAR in report:
        report_value: int = report[crk.ALERTNESS_OCULAR][crk.HA_VALUE]
        time_ago: float = report[crk.ALERTNESS_OCULAR][crk.TIME_AGO]
        assessment: str = get_alertness_ocular_assessment(report_value=report_value)

        row = get_time_proximal_row(gt, time_ago)

        if assessment == row[gtk.HA_OCULAR_ALERTNESS]:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_alertness_verbal(gt: pd.DataFrame, report: dict) -> float:
    """Scores the verbal alertness assessment from the report against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.ALERTNESS_VERBAL in report:
        report_value: int = report[crk.ALERTNESS_VERBAL][crk.HA_VALUE]
        time_ago: float = report[crk.ALERTNESS_VERBAL][crk.TIME_AGO]
        assessment: str = get_alertness_verbal_assessment(report_value=report_value)

        row = get_time_proximal_row(gt, time_ago)

        if assessment == row[gtk.HA_VERBAL_ALERTNESS]:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_alertness_motor(gt: pd.DataFrame, report: dict) -> float:
    """Scores the motor alertness assessment from the report against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report (dict): The casualty report

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.ALERTNESS_MOTOR in report:
        report_value: int = report[crk.ALERTNESS_MOTOR][crk.HA_VALUE]
        time_ago: float = report[crk.ALERTNESS_VERBAL][crk.TIME_AGO]
        assessment: str = get_alertness_motor_assessment(report_value=report_value)

        row = get_time_proximal_row(gt, time_ago)

        if assessment == row[gtk.HA_MOTOR_ALERTNESS]:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_heart_rate_update(gt: pd.DataFrame, report_update: dict) -> float:
    """Scores the heart rate assessment from a report update against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report_update (dict): The report update

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.HEART_RATE in report_update:
        hr_value: float = report_update[crk.HEART_RATE][crk.HA_VALUE]
        time_ago: float = report_update[crk.HEART_RATE][crk.TIME_AGO]

        row = get_time_proximal_row(gt, time_ago)

        if abs(hr_value - row[gtk.HEART_RATE]) < enums.Tolerance.HEART_RATE.value:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_respiration_rate_update(gt: pd.DataFrame, report_update: dict) -> float:
    """Scores the respiration rate assessment from a report update against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        report_update (dict): The report update

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.RESPIRATION_RATE in report_update:
        rr_value: float = report_update[crk.RESPIRATION_RATE][crk.HA_VALUE]
        time_ago: float = report_update[crk.RESPIRATION_RATE][crk.TIME_AGO]
        time_window: float = 60.0

        series = get_time_proximal_row(
            gt, time_ago, time_window
        )  # series will be used to compute average rr over given time window
        avg_rr: float = compute_avg_rr(series, time_ago, time_window)

        if abs(rr_value - avg_rr) < enums.Tolerance.RESPIRATION_RATE.value:
            return 1.0
        else:
            return -1.0
    else:
        return 0.0


def score_alertness_ocular_update(gt: pd.DataFrame, initial_report: dict, report_update: dict) -> float:
    """Scores the ocular alertness assessment from a report update against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        initial_report (dict): The initial report
        report_update (dict): The report update

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.ALERTNESS_OCULAR in report_update:
        report_update_value: int = report_update[crk.ALERTNESS_OCULAR][crk.HA_VALUE]
        time_ago: float = report_update[crk.ALERTNESS_OCULAR][crk.TIME_AGO]
        assessment: str = get_alertness_ocular_assessment(report_value=report_update_value)

        initial_report_value: int = initial_report[crk.ALERTNESS_OCULAR][crk.HA_VALUE]
        initial_assessment: str = get_alertness_ocular_assessment(initial_report_value)

        row = get_time_proximal_row(gt, time_ago)

        initial_assessment_correct: bool = score_alertness_ocular(gt=gt, report=initial_report) == 1.0
        assessment_correct: bool = assessment == row[gtk.HA_OCULAR_ALERTNESS]
        assessments_different: bool = assessment != initial_assessment

        if not assessment_correct:
            return -1.0

        return 1.0 if initial_assessment_correct and assessments_different else 0.0
    else:
        return 0.0


def score_alertness_verbal_update(gt: pd.DataFrame, initial_report: dict, report_update: dict) -> float:
    """Scores the verbal alertness assessment from a report update against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        initial_report (dict): The initial report
        report_update (dict): The report update

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.ALERTNESS_VERBAL in report_update:
        report_update_value: int = report_update[crk.ALERTNESS_VERBAL][crk.HA_VALUE]
        time_ago: float = report_update[crk.ALERTNESS_VERBAL][crk.TIME_AGO]
        assessment: str = get_alertness_verbal_assessment(report_value=report_update_value)

        initial_report_value: int = initial_report[crk.ALERTNESS_VERBAL][crk.HA_VALUE]
        initial_assessment: str = get_alertness_verbal_assessment(report_value=initial_report_value)

        row = get_time_proximal_row(gt, time_ago)

        initial_assessment_correct: bool = score_alertness_verbal(gt=gt, report=initial_report) == 1.0
        assessment_correct: bool = assessment == row[gtk.HA_VERBAL_ALERTNESS]
        assessments_different: bool = assessment != initial_assessment

        if not assessment_correct:
            return -1.0

        return 1.0 if initial_assessment_correct and assessments_different else 0.0
    else:
        return 0.0


def score_alertness_motor_update(gt: pd.DataFrame, initial_report: dict, report_update: dict):
    """Scores the motor alertness assessment from a report update against a ground truth casualty.

    Args:
        gt (pd.DataFrame): The ground truth casualty data
        initial_report (dict): The initial report
        report_update (dict): The report update

    Returns:
        float: The score based on correctness of the reported health assessment
    """
    if crk.ALERTNESS_MOTOR in report_update:
        report_update_value: int = report_update[crk.ALERTNESS_MOTOR][crk.HA_VALUE]
        time_ago: float = report_update[crk.ALERTNESS_VERBAL][crk.TIME_AGO]
        assessment: str = get_alertness_motor_assessment(report_value=report_update_value)

        initial_report_value: int = initial_report[crk.ALERTNESS_MOTOR][crk.HA_VALUE]
        initial_assessment: str = get_alertness_motor_assessment(report_value=initial_report_value)

        row = get_time_proximal_row(gt, time_ago)

        initial_assessment_correct: bool = score_alertness_motor(gt=gt, report=initial_report) == 1.0
        assessment_correct: bool = assessment == row[gtk.HA_MOTOR_ALERTNESS]
        assessments_different: bool = assessment != initial_assessment

        if not assessment_correct:
            return -1.0

        return 1.0 if initial_assessment_correct and assessments_different else 0.0
    else:
        return 0.0
    

def get_severe_hemorrhage_assessment(report_value: int) -> str:
    """Get the assessment for severe hemorrhage as a string based on the report value.

    Args:
        report_value (int): The reported value for severe hemorrhage

    Returns:
        str: The assessment that will be compared against the ground truth field for Severe Hemorrhage
    """
    if report_value == enums.SevereHemorrhage.ABSENT.value:
        return "absent"
    elif report_value == enums.SevereHemorrhage.PRESENT.value:
        return "present"
    

def get_respiratory_distress_assessment(report_value: int) -> str:
    """Get the assessment for respiratory distress as a string based on the report value.

    Args:
        report_value (int): The reported value for respiratory distress

    Returns:
        str: The assessment that will be compared against the ground truth field for Respiratory Distress
    """
    if report_value == enums.RespiratoryDistress.ABSENT.value:
        return "absent"
    elif report_value == enums.RespiratoryDistress.PRESENT.value:
        return "present"
    

def get_trauma_head_assessment(report_value: int) -> str:
    """Get the assessment for head trauma as a string based on the report value.

    Args:
        report_value (int): The reported value for head trauma

    Returns:
        str: The assessment that will be compared against the ground truth field for Head Trauma
    """
    if report_value == enums.TraumaHead.NORMAL.value:
        return "normal"
    elif report_value == enums.TraumaHead.WOUND.value:
        return "wound"
    elif report_value == enums.TraumaHead.NOT_TESTABLE.value:
        return "nt"
    

def get_trauma_torso_assessment(report_value: int) -> str:
    """Get the assessment for torso trauma as a string based on the report value.

    Args:
        report_value (int): The reported value for torso trauma

    Returns:
        str: The assessment that will be compared against the ground truth field for Torso Trauma
    """
    if report_value == enums.TraumaTorso.NORMAL.value:
        return "normal"
    elif report_value == enums.TraumaTorso.WOUND.value:
        return "wound"
    elif report_value == enums.TraumaTorso.NOT_TESTABLE.value:
        return "nt" 


def get_trauma_lower_ext_assessment(report_value: int) -> str:
    """Get the assessment for lower extremity trauma as a string based on the report value.

    Args:
        report_value (int): The reported value for lower extremity trauma

    Returns:
        str: The assessment that will be compared against the ground truth field for Lower Extremity Trauma
    """
    if report_value == enums.TraumaLowerExt.NORMAL.value:
        return "normal"
    elif report_value == enums.TraumaLowerExt.WOUND.value:
        return "wound"
    elif report_value == enums.TraumaLowerExt.AMPUTATION.value:
        return "amputation"
    elif report_value == enums.TraumaLowerExt.NOT_TESTABLE.value:
        return "nt" 


def get_trauma_upper_ext_assessment(report_value: int) -> str:
    """Get the assessment for upper extremity trauma as a string based on the report value.

    Args:
        report_value (int): The reported value for upper extremity trauma

    Returns:
        str: The assessment that will be compared against the ground truth field for Upper Extremity Trauma
    """
    if report_value == enums.TraumaUpperExt.NORMAL.value:
        return "normal"
    elif report_value == enums.TraumaUpperExt.WOUND.value:
        return "wound"
    elif report_value == enums.TraumaUpperExt.AMPUTATION.value:
        return "amputation"
    elif report_value == enums.TraumaUpperExt.NOT_TESTABLE.value:
        return "nt" 


def get_alertness_ocular_assessment(report_value: int) -> str:
    """Get the assessment for ocular alertness as a string based on the report value.

    Args:
        report_value (int): The reported value for ocular alertness

    Returns:
        str: The assessment that will be compared against the ground truth field for Ocular Alertness
    """
    if report_value == enums.AlertnessOcular.OPEN.value:
        return "open"
    elif report_value == enums.AlertnessOcular.CLOSED.value:
        return "closed"
    elif report_value == enums.AlertnessOcular.NOT_TESTABLE.value:
        return "nt"


def get_alertness_verbal_assessment(report_value: int) -> str:
    """Get the assessment for verbal alertness as a string based on the report value.

    Args:
        report_value (int): The reported value for verbal alertness

    Returns:
        str: The assessment that will be compared against the ground truth field for Verbal Alertness
    """
    if report_value == enums.AlertnessVerbal.NORMAL.value:
        return "normal"
    elif report_value == enums.AlertnessVerbal.ABNORMAL.value:
        return "abnormal"
    elif report_value == enums.AlertnessVerbal.ABSENT.value:
        return "absent"
    elif report_value == enums.AlertnessVerbal.NOT_TESTABLE.value:
        return "nt"


def get_alertness_motor_assessment(report_value: int) -> str:
    """Get the assessment for motor alertness as a string based on the report value.

    Args:
        report_value (int): The reported value for motor alertness

    Returns:
        str: The assessment that will be compared against the ground truth field for Motor Alertness
    """
    if report_value == enums.AlertnessMotor.NORMAL.value:
        return "normal"
    elif report_value == enums.AlertnessMotor.ABNORMAL.value:
        return "abnormal"
    elif report_value == enums.AlertnessMotor.ABSENT.value:
        return "absent"
    elif report_value == enums.AlertnessMotor.NOT_TESTABLE.value:
        return "nt"


def get_time_proximal_row(
    ground_truth: pd.DataFrame, time_ago: float, time_window: Optional[float] = None
) -> pd.Series | pd.DataFrame | None:
    """Gets a row as a Series or rows as a DataFrame from a ground truth based on the time window. If the time window extends behind the available rows, then the earliest becomes the lower extent of the time window.

    Args:
        ground_truth (pd.DataFrame): Contains ground truth data
        time_ago (float): Time in seconds relative to when a report was submitted
        time_window (Optional[float]): The extent of time in which to capture rows, starting at the assessment time and ending at the assessment time - time window

    Returns:
        pd.Series, pd.DataFrame, None: If no time window is provided, a Series, if a valid time window is provided, a DataFrame. If the assessment time is earlier than any available rows, then this assessment is considered invalid and None is returned.
    """
    # Get the row closest in time to time_ago
    end_idx = abs(ground_truth[gtk.TIME] - time_ago).idxmin()

    if ground_truth.iloc[end_idx][gtk.TIME] > time_ago:
        # The closest row was in the future, so step back to the "current" row
        end_idx -= 1

    if end_idx == -1:
        # time_ago was before any ground truth times
        return None

    if time_window is None:
        return ground_truth.iloc[end_idx]

    start_idx = end_idx

    while start_idx > 0 and ground_truth.iloc[start_idx][gtk.TIME] > round(time_ago - time_window, 2):
        start_idx -= 1

        if start_idx == 0:
            break

    return ground_truth.iloc[start_idx : end_idx + 1]


def compute_avg_rr(df: pd.DataFrame, time_ago: float, time_window: float) -> float:
    """Compute the average of the respiratory rates over the time window in seconds.

    Args:
        df (pd.DataFrame): DataFrame containing only the respiratory rates
        time_ago (float): The time in seconds relative to when a report was submitted, used here to compute the weight of each interval of respiration rates
        time_window(float): The divisor of the equation in which the average rate of respiration is computed

    Returns:
        float: The average respiratory rate
    """
    condensed_series = df.apply(lambda row: (row["Time"], row["RespirationRate"]), axis=1)
    reversed_series = condensed_series[::-1]

    sum_of_rates: float = 0.0
    curr_time = time_ago
    stop_time = max(time_ago - time_window, 0)

    for _, (time, respiration_rate) in reversed_series.items():
        time_step = curr_time - max(time, stop_time)
        if time_step < 0:  # `reversed_series` goes further back in time than necessary, so we're done
            break
        sum_of_rates += time_step * respiration_rate
        curr_time = time

    return sum_of_rates / (time_ago - stop_time)


# https://stackoverflow.com/a/73634275
class NpEncoder(json.JSONEncoder):
    def default(self, obj):
        dtypes = (np.datetime64, np.complexfloating)
        if isinstance(obj, dtypes):
            return str(obj)
        elif isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            if any([np.issubdtype(obj.dtype, i) for i in dtypes]):
                return obj.astype(str).tolist()
            return obj.tolist()
        return super(NpEncoder, self).default(obj)

def generate_final_report(
        out_path: str,
        score_tracker: ScoreTracker,
        reports: list[tuple[dict, enums.ReportRejection]],
        runtime: int):
    """Generates the final report starting with scored reports and followed by the list of all reports and the runtime.
    
    Args:
        out_path (str): The output path
        score_tracker (ScoreTracker): Tracks all validated reports and moves them through the scoring process
        reports (list[tuple[dict, enums.ReportRejection]]): A list of pairings of reports, scored and rejected, and the corresponding error
        runtime (int): The total runtime of the scenario
    """
    final_report: dict = {}
    final_report[frk.SCORED_CASUALTIES] = {}
    for cid in score_tracker.tracked_casualties:
        tc = score_tracker.tracked_casualties[cid]
        final_report[frk.SCORED_CASUALTIES][str(cid)] = {}
        final_report[frk.SCORED_CASUALTIES][str(cid)][frk.INITIAL] = {}
        final_report[frk.SCORED_CASUALTIES][str(cid)][frk.INITIAL][frk.REPORT] = tc.initial
        final_report[frk.SCORED_CASUALTIES][str(cid)][frk.INITIAL][frk.CORRECT] = get_correct_from_gt(tc.hungarian_gt_loc, score_tracker._gts, tc.initial)

        if tc.update is not None:
            final_report[frk.SCORED_CASUALTIES][str(cid)][frk.UPDATE] = {}
            final_report[frk.SCORED_CASUALTIES][str(cid)][frk.UPDATE][frk.REPORT] = tc.update
            final_report[frk.SCORED_CASUALTIES][str(cid)][frk.UPDATE][frk.CORRECT] = get_correct_from_gt(tc.hungarian_gt_loc, score_tracker._gts, tc.initial)

        final_report[frk.SCORED_CASUALTIES][str(cid)][frk.SCORE] = tc.hungarian_score[0]
        final_report[frk.SCORED_CASUALTIES][str(cid)][frk.GT_LOC] = { 
            frk.LATITUDE: tc.hungarian_gt_loc[1], 
            frk.LONGITUDE: tc.hungarian_gt_loc[0], 
            frk.ALTITUDE: tc.hungarian_gt_loc[2] 
        }
        final_report[frk.SCORED_CASUALTIES][str(cid)][frk.ERROR] = tc.hungarian_score[1].name if tc.hungarian_score[1] is not None else "None"
    final_report[frk.REPORTS] = []
    for report, err in reports:
        final_report[frk.REPORTS].append({ frk.REPORT: report, frk.ERROR: err.name if err is not None else "None" })
    final_report[frk.RUNTIME] = runtime
    final_report[frk.FINAL_SCORE] = sum(map(lambda tc: tc.hungarian_score[0], score_tracker.tracked_casualties.values()))
    logger.info(f"{final_report}")

    out_dir = os.path.dirname(out_path)
    pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as file:
        # The report may contain numpy types so we use NpEncoder to ensure those are serialized correctly
        json.dump(final_report, file, indent=4, cls=NpEncoder)


def get_correct_from_gt(
        gt_loc: tuple[float, float, float], 
        gts: list[tuple[tuple[float, float, float], pd.DataFrame]],
        report: dict) -> dict:
    """Generates a dict of correct health assessments from the ground truth data based on time reported in a casualty report
    
    Args:
        gt_loc (tuple[float, float, float]): The location of a ground truth casualty
        gts (list[tuple[tuple[float, float, float], pd.DataFrame]]): The mapping of locations to ground truth casualties
        report: A casualty report 

    Returns:
        dict: The correct health assessments from the ground truth data based on the associated report time for that health assessment
    """
    gt_dict: dict[tuple[float, float, float], pd.DataFrame] = dict(gts)
    gt = gt_dict[gt_loc]

    # The following health assessments utilize 'time_ago' from the reported location
    time_ago: float = report[crk.LOCATION][crk.TIME_AGO]
    row = get_time_proximal_row(ground_truth=gt, time_ago=time_ago)
    if crk.SEVERE_HEMORRHAGE in report:
        gt_severe_hemorrhage = row[gtk.HA_SEVERE_HEMORRHAGE]
    if crk.RESPIRATORY_DISTRESS in report:
        gt_respiratory_distress = row[gtk.HA_RESPIRATORY_DISTRESS]
    if crk.TRAUMA_HEAD in report:
        gt_trauma_head = row[gtk.HA_HEAD_TRAUMA]
    if crk.TRAUMA_TORSO in report:
        gt_trauma_torso = row[gtk.HA_TORSO_TRAUMA]
    if crk.TRAUMA_LOWER_EXTREMITY in report:
        gt_trauma_lower_ext = row[gtk.HA_LOWER_EXTREMITY_TRAUMA]
    if crk.TRAUMA_UPPER_EXTREMITY in report:
        gt_trauma_upper_ext = row[gtk.HA_UPPER_EXTREMITY_TRAUMA]

    # The following health assessments have their own 'time_ago' defined in the casualty report
    if crk.HEART_RATE in report:
        time_ago = report[crk.HEART_RATE][crk.TIME_AGO]
        row = get_time_proximal_row(ground_truth=gt, time_ago=time_ago)
        gt_heart_rate = row[gtk.HEART_RATE]

    if crk.RESPIRATION_RATE in report:
        time_ago = report[crk.RESPIRATION_RATE][crk.TIME_AGO]
        row = get_time_proximal_row(ground_truth=gt, time_ago=time_ago)
        gt_respiration_rate = row[gtk.RESPIRATION_RATE]

    if crk.CORE_TEMP in report:
        time_ago = report[crk.CORE_TEMP][crk.TIME_AGO]
        row = get_time_proximal_row(ground_truth=gt, time_ago=time_ago)
        gt_core_temperature = row[gtk.CORE_TEMP]

    if crk.ALERTNESS_OCULAR in report:
        time_ago = report[crk.ALERTNESS_OCULAR][crk.TIME_AGO]
        row = get_time_proximal_row(ground_truth=gt, time_ago=time_ago)
        gt_alertness_ocular = row[gtk.HA_OCULAR_ALERTNESS]

    if crk.ALERTNESS_VERBAL in report:
        time_ago = report[crk.ALERTNESS_VERBAL][crk.TIME_AGO]
        row = get_time_proximal_row(ground_truth=gt, time_ago=time_ago)
        gt_alertness_verbal = row[gtk.HA_VERBAL_ALERTNESS]

    if crk.ALERTNESS_MOTOR in report:
        time_ago = report[crk.ALERTNESS_MOTOR][crk.TIME_AGO]
        row = get_time_proximal_row(ground_truth=gt, time_ago=time_ago)
        gt_alertness_motor = row[gtk.HA_MOTOR_ALERTNESS]

    return {
        gtk.HA_SEVERE_HEMORRHAGE: gt_severe_hemorrhage,
        gtk.RESPIRATORY_DISTRESS: gt_respiratory_distress,
        gtk.HEART_RATE: gt_heart_rate,
        gtk.RESPIRATION_RATE: gt_respiration_rate,
        gtk.CORE_TEMP: gt_core_temperature,
        gtk.HA_HEAD_TRAUMA: gt_trauma_head,
        gtk.HA_TORSO_TRAUMA: gt_trauma_torso,
        gtk.HA_LOWER_EXTREMITY_TRAUMA: gt_trauma_lower_ext,
        gtk.HA_UPPER_EXTREMITY_TRAUMA: gt_trauma_upper_ext,
        gtk.HA_OCULAR_ALERTNESS: gt_alertness_ocular,
        gtk.HA_VERBAL_ALERTNESS: gt_alertness_verbal,
        gtk.HA_MOTOR_ALERTNESS: gt_alertness_motor
    }
