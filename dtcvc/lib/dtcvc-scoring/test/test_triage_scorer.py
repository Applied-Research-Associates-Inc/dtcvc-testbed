"""Unit tests for the scoring.py module."""

import json
import sys

import pandas as pd
import pytest

from triage_scorer import loading, scoring
from triage_scorer.constants import casualty_report_keys as crk
from triage_scorer.constants import enumerations as enums


@pytest.fixture
def ground_truths() -> list[tuple[tuple[float, float, float], pd.DataFrame]]:
    """Maps a GPS location (lat/lon/alt) to the ground truth data of a casualty.

    Returns:
        list: A list of mappings of casualty locations to ground truth data
    """
    casualty_loc_to_gt: list[tuple[tuple[float, float, float], pd.DataFrame]] = []
    scenario: dict | None = loading.load_scenario("./data/scenarios/scenario.example.yaml")
    maps: dict[str, dict[str, tuple[float, float, float]]] = loading.load_maps("./data/maps")
    gps_zone_locations: dict[str, tuple[float, float, float]] = {}
    ground_truth: dict[str, pd.DataFrame] = loading.load_ground_truth("./data/ground_truth")

    if scenario is not None:
        scenario_map_name: str = str(scenario["map"])

        for map_name in maps:
            if map_name in scenario_map_name:
                gps_zone_locations = maps[map_name]
                break

        scenario_casualties = scenario["casualties"]

        if gps_zone_locations:
            for casualty in scenario_casualties:
                zone: str = str(scenario_casualties[casualty]["zone"])
                zone_loc: tuple[float, float, float] = gps_zone_locations[zone]

                casualty_type = scenario_casualties[casualty]["casualty_type"]
                gt = ground_truth[casualty_type]

                casualty_loc_to_gt.append((zone_loc, gt))
    else:
        print("Failed to load scenario")
        sys.exit()

    return casualty_loc_to_gt


def test_track_report(ground_truths):
    cr_000: dict = read_report("./data/competitor_reports/cr_000_invalid_loc.json")
    cr_001: dict = read_report("./data/competitor_reports/cr_001.json")
    cr_002: dict = read_report("./data/competitor_reports/cr_002.json")
    cr_003: dict = read_report("./data/competitor_reports/cr_003.json")
    cr_004: dict = read_report("./data/competitor_reports/cr_004.json")
    cr_005: dict = read_report("./data/competitor_reports/cr_005.json")
    cr_001_update: dict = read_report("./data/competitor_reports/cr_001_update.json")
    cr_002_update: dict = read_report("./data/competitor_reports/cr_002_update.json")

    score_tracker = scoring.ScoreTracker(gts=ground_truths, limit=5)

    # Report cr_000: Test for report rejection where the reported location is outside the tolerance
    err = score_tracker.track_report(report=cr_000, report_time=60.0)
    assert err is None

    tc = score_tracker.tracked_casualties[cr_000[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score: float = 0.0
    assert score == expected_score
    assert err == enums.ReportRejection.NO_NEARBY_GT

    # Report cr_001: Test for report with full correctness and inside the Golden Window
    err = score_tracker.track_report(report=cr_001, report_time=60.0)
    assert err is None

    tc = score_tracker.tracked_casualties[cr_001[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score = 30.0
    assert score == expected_score
    assert err is None

    # Report cr_002: Test for report with full correctness and outside the Golden Window
    err = score_tracker.track_report(report=cr_002, report_time=60.0)
    assert err is None

    tc = score_tracker.tracked_casualties[cr_002[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score = 25.0
    assert score == expected_score
    assert err is None

    # Report cr_003: Test for report with full correctness and inside the Golden Window
    err = score_tracker.track_report(report=cr_003, report_time=60.0)
    assert err is None

    tc = score_tracker.tracked_casualties[cr_003[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score = 30.0
    assert score == expected_score
    assert err is None

    # Report cr_004: Test for report with full correctness and inside the Golden Window 
    err = score_tracker.track_report(report=cr_004, report_time=60.0)
    assert err is None 
    
    tc = score_tracker.tracked_casualties[cr_004[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score = 30.0
    assert score == expected_score
    assert err is None

    # Report cr_005: Test for report rejection where the report limit has been reached 
    err = score_tracker.track_report(report=cr_005, report_time=60.0)
    assert err == enums.ReportRejection.EXCESS_CASUALTY

    # Report update cr_001_update: Test report update with correctness 
    err = score_tracker.track_report(report=cr_001_update, report_time=60.0)
    assert err is None 

    tc = score_tracker.tracked_casualties[cr_001[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score = 34.0
    assert score == expected_score
    assert err is None

    # Report update cr_001_update: Test report rejection where report update already exists
    err = score_tracker.track_report(report=cr_001_update, report_time=60.0)
    assert err == enums.ReportRejection.EXCESS_UPDATE

    # Report mismatch for cr_002: Test report rejection where reported location for update report is outside the tolerance
    err = score_tracker.track_report(report=cr_002_update, report_time=60.0)
    assert err is None 

    tc = score_tracker.tracked_casualties[cr_002[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score = 25.0
    assert score == expected_score 
    assert err == enums.ReportRejection.UPDATE_GT_MISMATCH


def test_hungarian(ground_truths):
    score_tracker = scoring.ScoreTracker(gts=ground_truths, limit=5)
    cr_000: dict = read_report("./data/competitor_reports/cr_000_invalid_loc.json")
    score_tracker.track_report(report=cr_000, report_time=30.0)
    tc = score_tracker.tracked_casualties[cr_000[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score, expected_err = (0.0, enums.ReportRejection.NO_NEARBY_GT)
    assert score == expected_score
    assert err == expected_err

    cr_001: dict = read_report("./data/competitor_reports/cr_001.json")
    score_tracker.track_report(report=cr_001, report_time=60.0)
    tc = score_tracker.tracked_casualties[cr_001[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score, expected_err = (30.0, None)
    expected_gt_loc: tuple[float, float, float] = (8.110344, 48.926884, -30.419838)
    assert score == expected_score
    assert err == expected_err
    assert tc.hungarian_gt_loc == expected_gt_loc

    cr_002: dict = read_report("./data/competitor_reports/cr_002.json")
    score_tracker.track_report(report=cr_002, report_time=90.0)
    tc = score_tracker.tracked_casualties[cr_002[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score, expected_err = (25.0, None)
    expected_gt_loc: tuple[float, float, float] = (8.110252, 48.926889, -30.488405)
    assert score == expected_score
    assert err == expected_err
    assert tc.hungarian_gt_loc == expected_gt_loc

    cr_003: dict = read_report("./data/competitor_reports/cr_003.json")
    score_tracker.track_report(report=cr_003, report_time=120.0)
    tc = score_tracker.tracked_casualties[cr_003[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score, expected_err = (30.0, None)
    expected_gt_loc: tuple[float, float, float] = (8.110518, 48.926896, -30.544445)
    assert score == expected_score
    assert err == expected_err
    assert tc.hungarian_gt_loc == expected_gt_loc

    cr_004: dict = read_report("./data/competitor_reports/cr_004.json")
    score_tracker.track_report(report=cr_004, report_time=150.0)
    tc = score_tracker.tracked_casualties[cr_004[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score, expected_err = (30.0, None)
    expected_gt_loc: tuple[float, float, float] = (8.109992, 48.926891, -30.62149)
    assert score == expected_score
    assert err == expected_err
    assert tc.hungarian_gt_loc == expected_gt_loc


def test_hungarian_invalid_report_time(ground_truths):
    score_tracker = scoring.ScoreTracker(gts=ground_truths, limit=30)
    cr_000: dict = read_report("./data/competitor_reports/cr_000.json")
    err = score_tracker.track_report(report=cr_000, report_time=0.01)
    expected_err = enums.ReportRejection.INVALID_REPORT
    assert err == expected_err 


def test_hungarian_single_report(ground_truths):
    score_tracker = scoring.ScoreTracker(gts=ground_truths, limit=30)
    report: dict = {
        "casualty_id": 7,
        "team": "team-2",
        "system": "system-3",
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
    score_tracker.track_report(report=report, report_time=60.0)
    tc = score_tracker.tracked_casualties[report[crk.CASUALTY_ID]]
    score, err = tc.hungarian_score
    expected_score, expected_err = (30.0, None)
    assert score == expected_score
    assert err == expected_err


def test_score_report(ground_truths):
    loc_to_gt: tuple[tuple[float, float, float], pd.DataFrame] = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score, err = scoring.score_report(
        report_time=60.0, 
        loc_to_gt=loc_to_gt, 
        initial_report=report, 
        report_update=None
    )
    expected_score: float = 30.0
    assert score == expected_score
    assert err is None

    loc_to_gt = ground_truths[1]
    report = read_report("./data/competitor_reports/cr_002.json")
    score, err = scoring.score_report(
        report_time=15.0,
        loc_to_gt=loc_to_gt,
        initial_report=report,
        report_update=None
    )
    expected_score = 25.0
    assert score == expected_score 
    assert err is None 

    loc_to_gt = ground_truths[0]
    report = read_report("./data/competitor_reports/cr_001.json")
    report_update = read_report("./data/competitor_reports/cr_001_update.json")
    score, err = scoring.score_report(
        report_time=30.0,
        loc_to_gt=loc_to_gt,
        initial_report=report,
        report_update=report_update
    )
    expected_score = 34.0
    assert score == expected_score 
    assert err is None 

    # Testing point deduction 
    report = read_report("./data/competitor_reports/cr_001_incorrect.json")
    score, err = scoring.score_report(
        report_time=30.0,
        loc_to_gt=loc_to_gt,
        initial_report=report,
        report_update=None
    )
    expected_score = 7.0
    assert score == expected_score


def test_score_initial_report(ground_truths):
    loc_to_gt: tuple[tuple[float, float, float], pd.DataFrame] = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/initial_report.json")
    scored_points, bonus_points = scoring.score_initial_report(
        report_time=60.0, loc_to_gt=loc_to_gt, initial_report=report
    )
    expected_scored_points: float = 25.0
    assert scored_points == expected_scored_points

    expected_bonus_points: float = 5.0
    assert bonus_points == expected_bonus_points

    loc_to_gt = ground_truths[1]
    report = read_report("./data/competitor_reports/cr_002.json")
    scored_points, bonus_points = scoring.score_initial_report(
        report_time=15.0, loc_to_gt=loc_to_gt, initial_report=report
    )
    expected_scored_points = 25.0
    assert scored_points == expected_scored_points

    expected_bonus_points = 0.0
    assert bonus_points == expected_bonus_points


def test_score_report_update(ground_truths):
    loc_to_gt: tuple[tuple[float, float, float], pd.DataFrame] = ground_truths[0]
    initial_report: dict = read_report("./data/competitor_reports/initial_report.json")
    report_update: dict = read_report("./data/competitor_reports/report_update.json")

    # Test report update that is correct but where the Alertness assessments are unchanged from the initial report
    report_update_score: float = scoring.score_report_update(
        loc_to_gt=loc_to_gt, initial_report=initial_report, report_update=report_update
    )
    expected_score: float = 4.0
    assert report_update_score == expected_score

    # Test report update that receives full points 
    # Note: This test will fail if the corresponding CSV is updated
    report_update = read_report("./data/competitor_reports/report_update_full_points.json")
    report_update_score = scoring.score_report_update(
        loc_to_gt=loc_to_gt, initial_report=initial_report, report_update=report_update
    )
    expected_score = 7.0
    assert report_update_score == expected_score


def test_score_severe_hemorrhage(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/initial_report.json")

    # Test correct, in Golden Window
    scored_points, bonus_points = scoring.score_severe_hemorrhage(gt=gt, report=report, in_gw=True)
    expected_scored_points: float = 4.0
    assert scored_points == expected_scored_points

    expected_bonus_points: float = 2.0
    assert bonus_points == expected_bonus_points

    # Test correct, not in Golden Window
    scored_points, bonus_points = scoring.score_severe_hemorrhage(gt=gt, report=report, in_gw=False)
    expected_scored_points = 4.0
    assert scored_points == expected_scored_points

    expected_bonus_points = 0.0
    assert bonus_points == expected_bonus_points

    # Test incorrect
    report = create_initial_report(time_ago=30.0, severe_hemorrhage=1, respiratory_distress=1)
    scored_points, bonus_points = scoring.score_severe_hemorrhage(gt=gt, report=report, in_gw=True)
    expected_scored_points = -4.0
    assert scored_points == expected_scored_points

    expected_bonus_points = 0.0
    assert bonus_points == expected_bonus_points

    # Test not in report
    report = dict()
    scored_points, bonus_points = scoring.score_severe_hemorrhage(gt=gt, report=report, in_gw=False)
    expected_scored_points = 0.0
    assert scored_points == expected_scored_points

    expected_bonus_points = 0.0
    assert bonus_points == expected_bonus_points


def test_score_respiratory_distress(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")

    # Test correct, in Golden Window 
    scored_points, bonus_points = scoring.score_respiratory_distress(gt=gt, report=report, in_gw=True)
    expected_scored_points: float = 4.0
    assert scored_points == expected_scored_points

    expected_bonus_points: float = 2.0
    assert bonus_points == expected_bonus_points

    # Test correct, not in Golden Window
    scored_points, bonus_points = scoring.score_respiratory_distress(gt=gt, report=report, in_gw=False)
    expected_scored_points = 4.0
    assert scored_points == expected_scored_points

    expected_bonus_points = 0.0
    assert bonus_points == expected_bonus_points

    # Test incorrect
    report = create_initial_report(time_ago=30.0, severe_hemorrhage=0, respiratory_distress=0)
    scored_points, bonus_points = scoring.score_respiratory_distress(gt=gt, report=report, in_gw=True)
    expected_scored_points = -4.0
    assert scored_points == expected_scored_points

    expected_bonus_points = 0.0
    assert bonus_points == expected_bonus_points

    # Test not in report
    report = dict()
    scored_points, bonus_points = scoring.score_respiratory_distress(gt=gt, report=report, in_gw=False)
    expected_scored_points = 0.0
    assert scored_points == expected_scored_points

    expected_bonus_points = 0.0
    assert bonus_points == expected_bonus_points


def test_score_vitals(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    scored_points, bonus_points = scoring.score_vitals(
        gt=gt,
        report=report,
        in_gw=True
    )
    expected_scored_points: float = 3.0
    expected_bonus_points: float = 1.0
    assert scored_points == expected_scored_points
    assert bonus_points == expected_bonus_points

    scored_points, bonus_points = scoring.score_vitals(
        gt=gt,
        report=report,
        in_gw=False
    )
    expected_scored_points = 3.0
    expected_bonus_points = 0.0
    assert scored_points == expected_scored_points
    assert bonus_points == expected_bonus_points


def test_score_heart_rate(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score: float = scoring.score_heart_rate(gt=gt, report=report)
    expected_score: float = 1.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_heart_rate(gt=gt, report=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_heart_rate(gt=gt, report=report)
    expected_score = 0.0
    assert score == expected_score


def test_respiration_rate(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score: float = scoring.score_respiration_rate(gt=gt, report=report)
    expected_score: float = 1.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_respiration_rate(gt=gt, report=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_respiration_rate(gt=gt, report=report)
    expected_score = 0.0
    assert score == expected_score 


def test_score_score_temperature(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score: float = scoring.score_core_temperature(gt=gt, report=report)
    expected_score: float = 1.0
    assert score == expected_score 

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_core_temperature(gt=gt, report=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_core_temperature(gt=gt, report=report)
    expected_score = 0.0
    assert score == expected_score


def test_score_trauma_head(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score = scoring.score_trauma_head(gt=gt, report=report)
    expected_score: float = 1.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_trauma_head(gt=gt, report=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_trauma_head(gt=gt, report=report)
    expected_score = 0.0
    assert score == expected_score


def test_score_trauma_torso(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score = scoring.score_trauma_torso(gt=gt, report=report)
    expected_score: float = 1.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_trauma_torso(gt=gt, report=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_trauma_torso(gt=gt, report=report)
    expected_score = 0.0
    assert score == expected_score


def test_score_trauma_lower_ext(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score = scoring.score_trauma_lower_ext(gt=gt, report=report)
    expected_score: float = 1.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_trauma_lower_ext(gt=gt, report=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_trauma_lower_ext(gt=gt, report=report)
    expected_score = 0.0
    assert score == expected_score


def test_score_trauma_upper_ext(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score = scoring.score_trauma_upper_ext(gt=gt, report=report)
    expected_score: float = 1.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_trauma_upper_ext(gt=gt, report=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_trauma_upper_ext(gt=gt, report=report)
    expected_score = 0.0
    assert score == expected_score


def test_score_alertness(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score: float = scoring.score_alertness(gt=gt, report=report)
    expected_score: float = 4.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_alertness(gt=gt, report=report)
    expected_score = -3.0
    assert score == expected_score


def test_score_alertness_ocular(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score: float = scoring.score_alertness_ocular(gt=gt, report=report)
    expected_score: float = 1.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_alertness_ocular(gt=gt, report=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_alertness_ocular(gt=gt, report=report)
    expected_score = 0.0
    assert score == expected_score


def test_score_alertness_verbal(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score: float = scoring.score_alertness_verbal(gt=gt, report=report)
    expected_score: float = 1.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_alertness_verbal(gt=gt, report=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_alertness_verbal(gt=gt, report=report)
    expected_score = 0.0
    assert score == expected_score


def test_score_alertness_motor(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001.json")
    score: float = scoring.score_alertness_motor(gt=gt, report=report)
    expected_score: float = 1.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_incorrect.json")
    score = scoring.score_alertness_motor(gt=gt, report=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_alertness_motor(gt=gt, report=report)
    expected_score = 0.0
    assert score == expected_score


def test_score_heart_rate_update(ground_truths):
    _, gt = ground_truths[0]
    report: dict = read_report("./data/competitor_reports/cr_001_update.json")
    score: float = scoring.score_heart_rate_update(gt=gt, report_update=report)
    expected_score: float = 1.0
    assert score == expected_score

    report = read_report("./data/competitor_reports/cr_001_update_incorrect.json")
    score = scoring.score_heart_rate_update(gt=gt, report_update=report)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_heart_rate_update(gt=gt, report_update=report)
    expected_score = 0.0
    assert score == expected_score


def test_score_respiration_rate_update(ground_truths):
    _, gt = ground_truths[0]
    report_update: dict = read_report("./data/competitor_reports/cr_001_update.json")
    score: float = scoring.score_respiration_rate_update(gt=gt, report_update=report_update)
    expected_score: float = 1.0
    assert score == expected_score

    report_update = read_report("./data/competitor_reports/cr_001_update_incorrect.json")
    score = scoring.score_respiration_rate_update(gt=gt, report_update=report_update)
    expected_score = -1.0
    assert score == expected_score

    report = dict()
    score = scoring.score_respiration_rate_update(gt=gt, report_update=report)
    expected_score =0.0
    assert score == expected_score


def test_score_alertness_ocular_update(ground_truths):
    _, gt = ground_truths[0]
    initial_report: dict = read_report("./data/competitor_reports/cr_001.json")
    report_update: dict = read_report("./data/competitor_reports/report_update_full_points.json")
    score: float = scoring.score_alertness_ocular_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score: float = 1.0
    assert score == expected_score

    report_update = read_report("./data/competitor_reports/cr_001_update.json")
    score = scoring.score_alertness_ocular_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score = 0.0
    assert score == expected_score

    report_update = read_report("./data/competitor_reports/cr_001_update_incorrect.json")
    score = scoring.score_alertness_ocular_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score = -1.0
    assert score == expected_score

    report_update = dict()
    score = scoring.score_alertness_ocular_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score = 0.0
    assert score == expected_score


def test_score_alertness_verbal_update(ground_truths):
    _, gt = ground_truths[0]
    initial_report: dict = read_report("./data/competitor_reports/cr_001.json")
    report_update: dict = read_report("./data/competitor_reports/report_update_full_points.json")
    score: float = scoring.score_alertness_verbal_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score: float = 1.0
    assert score == expected_score 

    report_update = read_report("./data/competitor_reports/cr_001_update.json")
    score = scoring.score_alertness_verbal_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score = 0.0
    assert score == expected_score

    report_update = read_report("./data/competitor_reports/cr_001_update_incorrect.json")
    score = scoring.score_alertness_verbal_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score = -1.0
    assert score == expected_score
    
    report_update = dict()
    score = scoring.score_alertness_verbal_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score = 0.0
    assert score == expected_score 


def test_score_alertness_motor_update(ground_truths):
    _, gt = ground_truths[0]
    initial_report: dict = read_report("./data/competitor_reports/cr_001.json")
    report_update: dict = read_report("./data/competitor_reports/report_update_full_points.json")
    score: float = scoring.score_alertness_motor_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score: float = 1.0
    assert score == expected_score

    report_update = read_report("./data/competitor_reports/cr_001_update.json")
    score = scoring.score_alertness_motor_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score = 0.0
    assert score == expected_score

    report_update = read_report("./data/competitor_reports/cr_001_update_incorrect.json")
    score = scoring.score_alertness_motor_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score = -1.0
    assert score == expected_score

    report_update = dict()
    score = scoring.score_alertness_motor_update(gt=gt, initial_report=initial_report, report_update=report_update)
    expected_score = 0.0
    assert score == expected_score


def test_get_severe_hemorrhage_assessment(): 
    assessment: str = scoring.get_severe_hemorrhage_assessment(enums.SevereHemorrhage.ABSENT.value)
    assert assessment == "absent"

    assessment = scoring.get_severe_hemorrhage_assessment(enums.SevereHemorrhage.PRESENT.value)
    assert assessment == "present"


def test_get_respiratory_distress_assessment():
    assessment: str = scoring.get_respiratory_distress_assessment(enums.RespiratoryDistress.ABSENT.value)
    assert assessment == "absent"

    assessment = scoring.get_respiratory_distress_assessment(enums.RespiratoryDistress.PRESENT.value)
    assert assessment == "present"


def test_get_trauma_head_assessment():
    assessment: str = scoring.get_trauma_head_assessment(enums.TraumaHead.NORMAL.value)
    assert assessment == "normal"

    assessment = scoring.get_trauma_head_assessment(enums.TraumaHead.WOUND.value)
    assert assessment == "wound"

    assessment = scoring.get_trauma_head_assessment(enums.TraumaHead.NOT_TESTABLE.value)
    assert assessment == "nt"


def test_get_trauma_torso_assessment():
    assessment: str = scoring.get_trauma_torso_assessment(enums.TraumaTorso.NORMAL.value)
    assert assessment == "normal"

    assessment = scoring.get_trauma_torso_assessment(enums.TraumaTorso.WOUND.value)
    assert assessment == "wound"

    assessment = scoring.get_trauma_torso_assessment(enums.TraumaTorso.NOT_TESTABLE.value)
    assert assessment == "nt"


def test_get_trauma_lower_ext_assessment():
    assessment: str = scoring.get_trauma_lower_ext_assessment(enums.TraumaLowerExt.NORMAL.value)
    assert assessment == "normal"

    assessment = scoring.get_trauma_lower_ext_assessment(enums.TraumaLowerExt.WOUND.value)
    assert assessment == "wound"

    assessment = scoring.get_trauma_lower_ext_assessment(enums.TraumaLowerExt.AMPUTATION.value)
    assert assessment == "amputation"

    assessment = scoring.get_trauma_lower_ext_assessment(enums.TraumaLowerExt.NOT_TESTABLE.value)
    assert assessment == "nt"


def test_get_trauma_upper_ext_assessment():
    assessment: str = scoring.get_trauma_upper_ext_assessment(enums.TraumaUpperExt.NORMAL.value)
    assert assessment == "normal"

    assessment = scoring.get_trauma_upper_ext_assessment(enums.TraumaUpperExt.WOUND.value)
    assert assessment == "wound"

    assessment = scoring.get_trauma_upper_ext_assessment(enums.TraumaUpperExt.AMPUTATION.value)
    assert assessment == "amputation"

    assessment = scoring.get_trauma_upper_ext_assessment(enums.TraumaUpperExt.NOT_TESTABLE.value)
    assert assessment == "nt"


def test_get_alertness_ocular_assessment():
    assessment: str = scoring.get_alertness_ocular_assessment(enums.AlertnessOcular.OPEN.value)
    assert assessment == "open"

    assessment = scoring.get_alertness_ocular_assessment(enums.AlertnessOcular.CLOSED.value)
    assert assessment == "closed"

    assessment = scoring.get_alertness_ocular_assessment(enums.AlertnessOcular.NOT_TESTABLE.value)
    assert assessment == "nt"


def test_get_alertness_verbal_assessment():
    assessment: str = scoring.get_alertness_verbal_assessment(enums.AlertnessVerbal.NORMAL.value)
    assert assessment == "normal"

    assessment = scoring.get_alertness_verbal_assessment(enums.AlertnessVerbal.ABNORMAL.value)
    assert assessment == "abnormal"

    assessment = scoring.get_alertness_verbal_assessment(enums.AlertnessVerbal.ABSENT.value)
    assert assessment == "absent"

    assessment = scoring.get_alertness_verbal_assessment(enums.AlertnessVerbal.NOT_TESTABLE.value)
    assert assessment == "nt"


def test_get_alertness_motor_assessment():
    assessment: str = scoring.get_alertness_motor_assessment(enums.AlertnessMotor.NORMAL.value)
    assert assessment == "normal"

    assessment = scoring.get_alertness_motor_assessment(enums.AlertnessMotor.ABNORMAL.value)
    assert assessment == "abnormal"

    assessment = scoring.get_alertness_motor_assessment(enums.AlertnessMotor.ABSENT.value)
    assert assessment == "absent"

    assessment = scoring.get_alertness_motor_assessment(enums.AlertnessMotor.NOT_TESTABLE.value)
    assert assessment == "nt"


def read_report(filepath: str) -> dict:
    """Utility function used to read a report from the specified filepath."""
    with open(filepath) as file:
        report: dict = json.load(file)

    return report


def create_initial_report(
        time_ago: float, 
        severe_hemorrhage: int,
        respiratory_distress: int) -> dict:
    return {
        "casualty_id": 0,
        "team": "team-1",
        "system": "drone-1",
        "location": {"latitude": 48.926884, "longitude": 8.110344, "time_ago": time_ago},
        "severe_hemorrhage": severe_hemorrhage,
        "respiratory_distress": respiratory_distress,
        "hr": {"value": 75.0, "time_ago": 30.0},
        "rr": {"value": 13.0, "time_ago": 30.0},
        "temp": {"value": 36.5, "time_ago": 30.0},
        "trauma_head": 0,
        "trauma_torso": 0,
        "trauma_lower_ext": 0,
        "trauma_upper_ext": 0,
        "alertness_ocular": {"value": 0, "time_ago": 30.0},
        "alertness_verbal": {"value": 2, "time_ago": 30.0},
        "alertness_motor": {"value": 2, "time_ago": 30.0},
    }
