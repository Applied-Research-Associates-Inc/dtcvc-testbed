"""Unit tests for the module, /triage_scorer/scoring.py."""

import copy
import json
import sys
from collections import defaultdict, OrderedDict

import pandas as pd
import pytest

from triage_scorer import loading, scoring


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


@pytest.fixture
def full_pt_reps() -> list[OrderedDict]:
    """Returns a list of competitor reports where the expected scores would be partial points."""
    report_list: list[OrderedDict] = []

    full_pts_rep_alertness: str = "./data/competitor_reports/full_pt_reps/full_points_alertness.json"
    full_pts_rep_hem_res: str = "./data/competitor_reports/full_pt_reps/full_points_hem_res.json"
    full_pts_rep_trauma: str = "./data/competitor_reports/full_pt_reps/full_points_trauma.json"
    full_pts_rep_vitals: str = "./data/competitor_reports/full_pt_reps/full_points_vitals.json"

    report_list.append(read_report(full_pts_rep_alertness))
    report_list.append(read_report(full_pts_rep_hem_res))
    report_list.append(read_report(full_pts_rep_trauma))
    report_list.append(read_report(full_pts_rep_vitals))

    return report_list


@pytest.fixture
def partial_pt_reps() -> list[OrderedDict]:
    """Returns a list of competitor reports where the expected scores would be partial points."""
    report_list: list[OrderedDict] = []

    partial_points_report_alertness: str = "./data/competitor_reports/partial_pt_reps/partial_points_alertness.json"
    partial_points_report_hem_res: str = "./data/competitor_reports/partial_pt_reps/partial_points_hem_res.json"
    partial_points_report_trauma: str = "./data/competitor_reports/partial_pt_reps/partial_points_trauma.json"
    partial_points_report_vitals: str = "./data/competitor_reports/partial_pt_reps/partial_points_vitals.json"

    report_list.append(read_report(partial_points_report_alertness))
    report_list.append(read_report(partial_points_report_hem_res))
    report_list.append(read_report(partial_points_report_trauma))
    report_list.append(read_report(partial_points_report_vitals))

    return report_list


@pytest.fixture
def no_pt_reps() -> list[OrderedDict]:
    """Returns a list of competitor reports where the expected scores would be no points."""
    report_list: list[OrderedDict] = []

    no_points_report_alertness: str = "./data/competitor_reports/no_pt_reps/no_points_alertness.json"
    no_points_report_hem_res: str = "./data/competitor_reports/no_pt_reps/no_points_hem_res.json"
    no_points_report_trauma: str = "./data/competitor_reports/no_pt_reps/no_points_trauma.json"
    no_points_report_vitals: str = "./data/competitor_reports/no_pt_reps/no_points_vitals.json"

    report_list.append(read_report(no_points_report_alertness))
    report_list.append(read_report(no_points_report_hem_res))
    report_list.append(read_report(no_points_report_trauma))
    report_list.append(read_report(no_points_report_vitals))

    return report_list


def read_report(filename: str) -> OrderedDict:
    """Utility function used to read a report."""
    with open(filename) as file:
        report: OrderedDict = json.load(file, object_pairs_hook=OrderedDict)

    return report


def create_aggregate(reports: list[OrderedDict], report_time: int):
    """Helper function used in the test_cate_aggregation() unit test.

    Args:
        reports (list[OrderedDict]): List of ordered reports
        report_time (int): Start time for reports. Incremented by 1 per report that is tracked.

    Returns:
        dict: The aggregate of a CasualtyAssessmentTracker
    """
    cat = scoring.CasualtyAssessmentTracker()

    for report in reports:
        cat.track_report(report, report_time)
        report_time += 1

    return cat._aggregate


def test_unscored_cat_aggregation(full_pt_reps, partial_pt_reps, no_pt_reps):
    """Runs three tests for aggregation of the CasualtyAssessmentTracker.

    Args:
        full_pt_reps (list[OrderedDict]): Reports where full points are expected.
        partial_pt_reps (list[OrderedDict]): Reports where partial points are expected.
        no_pt_reps (list[OrderedDict]): Reports where no points are expected.
    """
    report_start_time = 30

    # Define an aggregate based on full points reports
    fmtted_full_pt_aggregate = {
        "vitals": {
            "heart_rate": 75.0,
            "respiration_rate": 13.0,
        },
        "injuries": {
            "severe_hemorrhage": False,
            "respiratory_distress": True,
            "trauma": {
                "head": "normal",
                "torso": "normal",
                "upper_extremity": "normal",
                "lower_extremity": "normal",
            },
            "alertness": {
                "ocular": "open",
                "verbal": "absent",
                "motor": "absent",
            },
        },
    }

    # Order the reports so that the full points reports are at the end of the list
    reports = partial_pt_reps + no_pt_reps + full_pt_reps
    aggregate = create_aggregate(reports=reports, report_time=report_start_time)

    # Format the aggregate so that the values of each ScorableField are exposed
    assert scoring.format_aggregate(aggregate, scored=False) == fmtted_full_pt_aggregate

    # Define an aggregate based on partial points reports
    fmtted_partial_pt_aggregate = {
        "vitals": {
            "heart_rate": 75,
            "respiration_rate": 20,
        },
        "injuries": {
            "severe_hemorrhage": False,
            "respiratory_distress": False,
            "trauma": {
                "head": "normal",
                "torso": "normal",
                "upper_extremity": "wound",
                "lower_extremity": "wound",
            },
            "alertness": {
                "ocular": "open",
                "verbal": "normal",
                "motor": "absent",
            },
        },
    }

    # Order the reports so that the partial points reports are at the end of the list
    reports = full_pt_reps + no_pt_reps + partial_pt_reps
    aggregate = create_aggregate(reports=reports, report_time=report_start_time)

    assert scoring.format_aggregate(aggregate, scored=False) == fmtted_partial_pt_aggregate

    # Define an aggregate based on no points reports
    fmtted_no_pt_aggregate = {
        "vitals": {
            "heart_rate": 120,
            "respiration_rate": 20,
        },
        "injuries": {
            "severe_hemorrhage": True,
            "respiratory_distress": False,
            "trauma": {
                "head": "wound",
                "torso": "wound",
                "upper_extremity": "wound",
                "lower_extremity": "wound",
            },
            "alertness": {
                "ocular": "absent",
                "verbal": "normal",
                "motor": "open",
            },
        },
    }

    # Order the reports so that the no points reports are at the end of the list
    reports = full_pt_reps + partial_pt_reps + no_pt_reps
    aggregate = create_aggregate(reports=reports, report_time=report_start_time)

    assert scoring.format_aggregate(aggregate, scored=False) == fmtted_no_pt_aggregate


def test_scored_cat_aggregation(ground_truths):
    """Tests the formatting of scored CasualtyAssessmentTrackers.

    Scoring adds metadata to scorable fields such as which GT was used as well as the time the field was
    received and when it should be assessed against.

    Args:
        ground_truths (list[tuple[tuple[float, float, float], pandas.DataFrame]]): The list of ground truth mappings
    """
    report_start_time = 30

    # Order matters here for the test since the report time is incremented by 1 for each report ingested in this test
    reports = list(
        map(
            read_report,
            [
                "./data/competitor_reports/test_scored_cat_aggregation/full_points_alertness.json",
                "./data/competitor_reports/test_scored_cat_aggregation/full_points_hem_res.json",
                "./data/competitor_reports/test_scored_cat_aggregation/full_points_trauma.json",
                "./data/competitor_reports/test_scored_cat_aggregation/full_points_vitals.json",
            ],
        )
    )
    cat = scoring.CasualtyAssessmentTracker()
    for i in range(len(reports)):
        cat.track_report(reports[i], report_start_time + i)

    # Score for aggregate side effects
    scoring.score_casualty(cat=cat, gts=ground_truths, gts_used=set())

    fmtted_full_pt_aggregate = {
        "vitals": {
            "heart_rate": {
                "value": 75.0,
                "assessment_time": 30,
                "receipt_time": report_start_time + 3,
                "correct": True,
                "gt_name": "VTCScenario004_TP_Results_PostProcessed.csv",
            },
            "respiration_rate": {
                "value": 14.0,
                "assessment_time": 30,
                "receipt_time": report_start_time + 3,
                "correct": True,
                "gt_name": "VTCScenario004_TP_Results_PostProcessed.csv",
            },
        },
        "injuries": {
            "severe_hemorrhage": {
                "value": False,
                "assessment_time": 30,
                "receipt_time": report_start_time + 1,
                "correct": True,
                "gt_name": "VTCScenario002_LA_Results_PostProcessed.csv",
            },
            "respiratory_distress": {
                "value": False,
                "assessment_time": 30,
                "receipt_time": report_start_time + 1,
                "correct": True,
                "gt_name": "VTCScenario002_LA_Results_PostProcessed.csv",
            },
            "trauma": {
                "head": {
                    "value": "normal",
                    "assessment_time": 30,
                    "receipt_time": report_start_time + 2,
                    "correct": True,
                    "gt_name": "VTCScenario003_BU_Results_PostProcessed.csv",
                },
                "torso": {
                    "value": "normal",
                    "assessment_time": 30,
                    "receipt_time": report_start_time + 2,
                    "correct": True,
                    "gt_name": "VTCScenario003_BU_Results_PostProcessed.csv",
                },
                "upper_extremity": {
                    "value": "normal",
                    "assessment_time": 30,
                    "receipt_time": report_start_time + 2,
                    "correct": True,
                    "gt_name": "VTCScenario003_BU_Results_PostProcessed.csv",
                },
                "lower_extremity": {
                    "value": "wound",
                    "assessment_time": 30,
                    "receipt_time": report_start_time + 2,
                    "correct": True,
                    "gt_name": "VTCScenario003_BU_Results_PostProcessed.csv",
                },
            },
            "alertness": {
                "ocular": {
                    "value": "open",
                    "assessment_time": 30,
                    "receipt_time": report_start_time,
                    "correct": True,
                    "gt_name": "VTCScenario001_TP_Results_PostProcessed.csv",
                },
                "verbal": {
                    "value": "absent",
                    "assessment_time": 30,
                    "receipt_time": report_start_time,
                    "correct": True,
                    "gt_name": "VTCScenario001_TP_Results_PostProcessed.csv",
                },
                "motor": {
                    "value": "absent",
                    "assessment_time": 30,
                    "receipt_time": report_start_time,
                    "correct": True,
                    "gt_name": "VTCScenario001_TP_Results_PostProcessed.csv",
                },
            },
        },
    }

    assert scoring.format_aggregate(cat._aggregate, scored=True) == fmtted_full_pt_aggregate


def test_full_pts_score_gw(ground_truths, full_pt_reps):
    """Tests scoring reports inside the golden window where full points are expected.

    Args:
        ground_truths (list[tuple[tuple[float, float, float], pandas.DataFrame]]): The list of ground truth mappings
        full_pt_reps (list[OrderedDict]): Reports where full points are expected
    """
    cat = scoring.CasualtyAssessmentTracker()
    for report in full_pt_reps:
        cat.track_report(report, 30.0)

    expected_score: float = 15.0
    score, _ = scoring.score_casualty(cat=cat, gts=ground_truths, gts_used=set())

    assert score == expected_score


def test_partial_pts_score_gw(ground_truths, partial_pt_reps):
    """Tests scoring reports inside the golden window where partial points are expected.

    Args:
        ground_truths (list[tuple[tuple[float, float, float], pandas.DataFrame]]): The list of ground truth mappings
        partial_pt_reps (list[OrderedDict]): Reports where partial points are expected
    """
    cat = scoring.CasualtyAssessmentTracker()
    for report in partial_pt_reps:
        cat.track_report(report, 30)

    expected_score: float = 7.0
    score, _ = scoring.score_casualty(cat=cat, gts=ground_truths, gts_used=set())

    assert score == expected_score


def test_no_pts_score_gw(ground_truths, no_pt_reps):
    """Tests scoring reports inside the golden window where no points are expected.

    Args:
        ground_truths (list[tuple[tuple[float, float, float], pandas.DataFrame]]): The list of ground truth mappings
        no_pt_reps (list[OrderedDict]): Reports where no no are expected
    """
    cat = scoring.CasualtyAssessmentTracker()
    for report in no_pt_reps:
        cat.track_report(report, 30)

    expected_score: float = 0.0
    score, _ = scoring.score_casualty(cat=cat, gts=ground_truths, gts_used=set())

    assert score == expected_score


def test_full_pts_score(ground_truths, full_pt_reps):
    """Tests scoring reports outside the golden window where full points are expected.

    Args:
        ground_truths (list[tuple[tuple[float, float, float], pandas.DataFrame]]): The list of ground truth mappings
        full_pt_reps (list[OrderedDict]): Reports where full points are expected
    """
    cat = scoring.CasualtyAssessmentTracker()
    for report in full_pt_reps:
        cat.track_report(report, scoring.golden_window + 1)

    expected_score: float = 10.0
    score, _ = scoring.score_casualty(cat=cat, gts=ground_truths, gts_used=set())

    assert score == expected_score


def test_partial_pts_score(ground_truths, partial_pt_reps):
    """Tests scoring reports outside the golden window where partial points are expected.

    Args:
        ground_truths (list[tuple[tuple[float, float, float], pandas.DataFrame]]): The list of ground truth mappings
        partial_pt_reps (list[OrderedDict]): Reports where partial points are expected
    """
    cat = scoring.CasualtyAssessmentTracker()
    for report in partial_pt_reps:
        cat.track_report(report, scoring.golden_window + 1)

    expected_score: float = 5.0
    score, _ = scoring.score_casualty(cat=cat, gts=ground_truths, gts_used=set())

    assert score == expected_score


def test_no_pts_score(ground_truths, no_pt_reps):
    """Tests scoring reports outside the golden window where no points are expected.

    Args:
        ground_truths (list[tuple[tuple[float, float, float], pandas.DataFrame]]): Mapping of casualty locations to ground truth data
        no_pt_reps (list[OrderedDict]): Reports where no points are expected
    """
    cat = scoring.CasualtyAssessmentTracker()
    for report in no_pt_reps:
        cat.track_report(report, scoring.golden_window + 1)

    expected_score: float = 0.0
    score, _ = scoring.score_casualty(cat=cat, gts=ground_truths, gts_used=set())

    assert score == expected_score


def test_generate_final_score_and_casualty_assessments(ground_truths):
    """Tests the summation of the scores for each aggregate from a casualty ID.

    Args:
        ground_truths (list[tuple[tuple[float, float, float], pandas.DataFrame]]): Mapping of casualty locations to ground truth data
    """
    cats: dict[str, scoring.CasualtyAssessmentTracker] = defaultdict(lambda: scoring.CasualtyAssessmentTracker())

    # forward
    # orange is Casualty1, blue is Casualty2 in the ground truths
    # - good_orange_vitals (3)
    # - bad_orange_trauma (4) - coincidental Casualty1/Casualty2 overlap for a point; this uses up blue
    # - good_blue_full (4) - blue got used up by bad_orange_trauma, so no new points
    # - good_orange_trauma (5+15) - good_orange_trauma overwrites bad_orange_trauma for aggregate, thus blue is available

    report: OrderedDict = read_report(
        "./data/competitor_reports/test_generate_final_score_and_casualty_assessments/good_orange_vitals.json"
    )
    cat: scoring.CasualtyAssessmentTracker = cats[report["casualty_id"]]
    cat.track_report(report, scoring.golden_window)
    final_score, _ = scoring.generate_final_score_and_casualty_assessments(cats=cats, gts=ground_truths)
    assert final_score == 3.0

    report = read_report(
        "./data/competitor_reports/test_generate_final_score_and_casualty_assessments/bad_orange_trauma.json"
    )
    cat = cats[report["casualty_id"]]
    cat.track_report(report, scoring.golden_window)
    final_score, _ = scoring.generate_final_score_and_casualty_assessments(cats=cats, gts=ground_truths)
    assert final_score == 4.0

    report = read_report(
        "./data/competitor_reports/test_generate_final_score_and_casualty_assessments/good_blue_full.json"
    )
    cat = cats[report["casualty_id"]]
    cat.track_report(report, scoring.golden_window)
    final_score, _ = scoring.generate_final_score_and_casualty_assessments(cats=cats, gts=ground_truths)
    assert final_score == 4.0

    report = read_report(
        "./data/competitor_reports/test_generate_final_score_and_casualty_assessments/good_orange_trauma.json"
    )
    cat = cats[report["casualty_id"]]
    cat.track_report(report, scoring.golden_window)
    final_score, _ = scoring.generate_final_score_and_casualty_assessments(cats=cats, gts=ground_truths)
    assert final_score == 20.0

    # Reset cats to do the reverse order
    cats: dict[str, scoring.CasualtyAssessmentTracker] = defaultdict(lambda: scoring.CasualtyAssessmentTracker())

    # reverse
    # - good_blue_full (15)
    # - good_orange_trauma (17)
    # - bad_orange_trauma (15)
    # - good_orange_vitals (18)

    report: OrderedDict = read_report(
        "./data/competitor_reports/test_generate_final_score_and_casualty_assessments/good_blue_full.json"
    )
    cat: scoring.CasualtyAssessmentTracker = cats[report["casualty_id"]]
    cat.track_report(report, scoring.golden_window)
    final_score, _ = scoring.generate_final_score_and_casualty_assessments(cats=cats, gts=ground_truths)
    assert final_score == 15.0

    report = read_report(
        "./data/competitor_reports/test_generate_final_score_and_casualty_assessments/good_orange_trauma.json"
    )
    cat = cats[report["casualty_id"]]
    cat.track_report(report, scoring.golden_window)
    final_score, _ = scoring.generate_final_score_and_casualty_assessments(cats=cats, gts=ground_truths)
    assert final_score == 17.0

    report = read_report(
        "./data/competitor_reports/test_generate_final_score_and_casualty_assessments/bad_orange_trauma.json"
    )
    cat = cats[report["casualty_id"]]
    cat.track_report(report, scoring.golden_window)
    final_score, _ = scoring.generate_final_score_and_casualty_assessments(cats=cats, gts=ground_truths)
    assert final_score == 15.0

    report = read_report(
        "./data/competitor_reports/test_generate_final_score_and_casualty_assessments/good_orange_vitals.json"
    )
    cat.track_report(report, scoring.golden_window)
    final_score, _ = scoring.generate_final_score_and_casualty_assessments(cats=cats, gts=ground_truths)
    assert final_score == 18.0


def test_score_casualty(ground_truths):
    """This test is sort of a larger scale test of aggregation, but is specifically checking that GTs get "used" correctly.

    Args:
        ground_truths (list[tuple[tuple[float, float, float], pandas.DataFrame]]): Mapping of casualty locations to ground truth data
    """
    orange_loc: tuple[float, float, float] = (8.110344, 48.926884, -30.419838)
    blue_loc: tuple[float, float, float] = (8.110252, 48.926889, -30.488405)

    cat: scoring.CasualtyAssessmentTracker = scoring.CasualtyAssessmentTracker()
    gts_used: set[tuple[float, float, float]] = set()

    score: float = 0.0

    vitals_report: OrderedDict = read_report("./data/competitor_reports/test_score_casualty/good_orange_vitals.json")
    cat.track_report(vitals_report, scoring.golden_window)
    score, casualty_gts_used = scoring.score_casualty(cat=cat, gts=ground_truths, gts_used=gts_used)

    expected_score: float = 3.0
    assert score == expected_score
    assert casualty_gts_used == set([orange_loc])

    # This report is mapped to blue location, but the report contains an assessment for orange
    bad_trauma_report: OrderedDict = read_report("./data/competitor_reports/test_score_casualty/bad_orange_trauma.json")
    cat.track_report(bad_trauma_report, scoring.golden_window)
    score, casualty_gts_used = scoring.score_casualty(cat=cat, gts=ground_truths, gts_used=gts_used)

    expected_score = 4.0
    assert score == expected_score
    assert casualty_gts_used == set([orange_loc, blue_loc])

    # This report maps to orange location and overwrite bad_trauma_report
    good_trauma_report: OrderedDict = read_report(
        "./data/competitor_reports/test_score_casualty/good_orange_trauma.json"
    )
    cat.track_report(good_trauma_report, scoring.golden_window)
    score, casualty_gts_used = scoring.score_casualty(cat=cat, gts=ground_truths, gts_used=gts_used)

    expected_score = 5.0
    assert score == expected_score
    assert casualty_gts_used == set([orange_loc])


def test_get_space_proximal_gt(ground_truths):
    """Test the return values of get_space_proximal_gt() by comparing a tuple of floats and a sample from the DataFrame.

    Args:
        ground_truths (list[tuple[tuple[float, float, float], pandas.DataFrame]]): Mapping of casualty locations to ground truth data
    """
    sf: scoring.ScorableField = scoring.ScorableField(
        assessment_time=30.0,
        report_time=900.0,
        lon=8.110344,
        lat=48.926884,
        alt=-30.419838,
        value=0.0,
    )

    location, df = scoring.get_space_proximal_gt(ground_truths=ground_truths, sf=sf)
    # We will only test a sample from the DataFrame
    df_sample: pd.DataFrame = df[:21]

    expected_location: tuple[float, float, float] = (8.110344, 48.926884, -30.419838)
    # Build a DataFrame by hand but only sample the first 300 seconds of data (21 rows)
    expected_data: dict = {
        "Time": [
            0.02,
            15.02,
            30.02,
            45.02,
            60.02,
            75.02,
            90.02,
            105.02,
            120.02,
            135.02,
            150.02,
            165.02,
            180.02,
            195.02,
            210.02,
            225.02,
            240.02,
            255.02,
            270.02,
            285.02,
            300.02,
        ],
        "HeartRate": [
            72.00,
            75.19,
            84.15,
            88.83,
            90.79,
            91.32,
            91.76,
            92.16,
            92.43,
            92.72,
            93.23,
            94.28,
            95.29,
            95.74,
            96.40,
            96.70,
            97.81,
            98.74,
            99.31,
            100.03,
            101.89,
        ],
        "RespirationRate": [
            13.64,
            13.70,
            13.45,
            13.16,
            12.93,
            12.93,
            13.27,
            13.76,
            14.49,
            15.23,
            15.71,
            16.30,
            17.05,
            17.54,
            18.18,
            18.63,
            19.11,
            19.61,
            20.13,
            20.69,
            21.13,
        ],
        "severe_hemorrhage": [
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
        ],
        "respiratory_distress": [
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
            "present",
        ],
        "trauma_head": [
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
        ],
        "trauma_torso": [
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
        ],
        "trauma_upper_ext": [
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
        ],
        "trauma_lower_ext": [
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
            "normal",
        ],
        "alertness_ocular": [
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
            "open",
        ],
        "alertness_verbal": [
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
        ],
        "alertness_motor": [
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
            "absent",
        ],
    }
    expected_df: pd.DataFrame = pd.DataFrame(data=expected_data)

    assert location == expected_location
    assert df_sample.equals(expected_df)

    sf.lon = 8.110252
    sf.lat = 48.926889
    sf.alt = -30.488405
    # Get a new DataFrame from get_space_proximal_gt() to test against the hand-crafted DataFrame
    location, df = scoring.get_space_proximal_gt(ground_truths=ground_truths, sf=sf)

    expected_location = (8.110252, 48.926889, -30.488405)
    assert location == expected_location
    assert df.equals(expected_df) == False


def test_get_time_proximal_row__no_tw(ground_truths):
    sf: scoring.ScorableField = scoring.ScorableField(
        assessment_time=30.0,
        report_time=900.0,
        lon=8.110344,
        lat=48.926884,
        alt=-30.419838,
        value=0.0,
    )

    _, gt = scoring.get_space_proximal_gt(ground_truths=ground_truths, sf=sf)

    row: pd.Series = scoring.get_time_proximal_row(ground_truth=gt, sf=sf)

    expected_data: dict = {
        "Time": 15.02,
        "HeartRate": 75.19,
        "RespirationRate": 13.7,
        "severe_hemorrhage": "absent",
        "respiratory_distress": "present",
        "trauma_head": "normal",
        "trauma_torso": "normal",
        "trauma_upper_ext": "normal",
        "trauma_lower_ext": "normal",
        "alertness_ocular": "open",
        "alertness_verbal": "absent",
        "alertness_motor": "absent",
    }

    expected_row: pd.Series = pd.Series(data=expected_data, name=1)
    assert row.equals(expected_row)

    sf.assessment_time = 15.02
    row: pd.Series = scoring.get_time_proximal_row(ground_truth=gt, sf=sf)
    assert row.equals(expected_row)

    # Get another row to confirm that it wasn't a fluke
    sf.assessment_time = 120.02
    row: pd.Series = scoring.get_time_proximal_row(ground_truth=gt, sf=sf)
    assert not row.equals(expected_row)

    # Ground truths start at 0.02 (that's a BioGears thing)
    sf.assessment_time = 0.0
    rows: pd.DataFrame = scoring.get_time_proximal_row(ground_truth=gt, sf=sf)
    assert rows is None


def test_get_time_proximal_row__tw(ground_truths):
    sf: scoring.ScorableField = scoring.ScorableField(
        assessment_time=33.0,
        report_time=900.0,
        lon=8.110344,
        lat=48.926884,
        alt=-30.419838,
        value=0.0,
    )

    _, gt = scoring.get_space_proximal_gt(ground_truths=ground_truths, sf=sf)

    rows: pd.DataFrame = scoring.get_time_proximal_row(ground_truth=gt, sf=sf, time_window=30)

    expected_rows: pd.DataFrame = gt[:3]
    assert rows.equals(expected_rows)

    # Test for when the tw would extend behind the rows available
    rows = scoring.get_time_proximal_row(ground_truth=gt, sf=sf, time_window=45.0)
    assert rows.equals(expected_rows)

    sf.assessment_time = 90.02
    rows = scoring.get_time_proximal_row(ground_truth=gt, sf=sf, time_window=60.0)

    expected_rows = gt[2:7]
    assert rows.equals(expected_rows)

    # Test for tw not a multiple of 15
    rows = scoring.get_time_proximal_row(ground_truth=gt, sf=sf, time_window=50.0)
    assert rows.equals(expected_rows)

    sf.assessment_time = 0.0
    rows: pd.DataFrame = scoring.get_time_proximal_row(ground_truth=gt, sf=sf, time_window=60.0)
    assert rows is None

    # Test for case where end is first row
    sf.assessment_time = 1.0
    expected_rows = gt[0:1]
    rows = scoring.get_time_proximal_row(ground_truth=gt, sf=sf, time_window=50)
    assert rows.equals(expected_rows)


def test_compute_avg_rr(ground_truths):
    sf: scoring.ScorableField = scoring.ScorableField(
        assessment_time=75.02,
        report_time=900.0,
        lon=8.110344,
        lat=48.926884,
        alt=-30.419838,
        value=0.0,
    )

    _, gt = scoring.get_space_proximal_gt(ground_truths=ground_truths, sf=sf)

    time_window: float = 60.0
    rows: pd.DataFrame = scoring.get_time_proximal_row(ground_truth=gt, sf=sf, time_window=time_window)

    avg_rr: float = scoring.compute_avg_rr(df=rows, assessment_time=sf.assessment_time, time_window=time_window)
    expected_avg_rr: float = 13.31
    assert round(avg_rr, 2) == expected_avg_rr

    sf.assessment_time = 80.0
    time_window = 60.0
    rows = scoring.get_time_proximal_row(ground_truth=gt, sf=sf, time_window=time_window)

    avg_rr = scoring.compute_avg_rr(df=rows, assessment_time=sf.assessment_time, time_window=time_window)
    expected_avg_rr = 13.25
    assert round(avg_rr, 2) == expected_avg_rr

    rows = scoring.get_time_proximal_row(ground_truth=gt, sf=sf, time_window=120.0)

    avg_rr = scoring.compute_avg_rr(df=rows, assessment_time=sf.assessment_time, time_window=60.0)
    expected_avg_rr = 13.25
    assert round(avg_rr, 2) == expected_avg_rr

    time_window = 120.0
    rows = scoring.get_time_proximal_row(ground_truth=gt, sf=sf, time_window=time_window)

    # Assessment time is still 80.0
    avg_rr = scoring.compute_avg_rr(df=rows, assessment_time=sf.assessment_time, time_window=time_window)
    expected_avg_rr = 13.34
    assert round(avg_rr, 2) == expected_avg_rr


def test_phase1_multianswer(ground_truths):
    report_base = {
        "observation_start": 15.0,
        "observation_end": 30.0,
        "assessment_time": 30.0,
        "casualty_id": 0,
        "drone_id": 10,
        "location": {"lon": 0, "lat": 0, "alt": 0},
        "vitals": {},
        "injuries": {"alertness": {"ocular": "open"}},
    }

    # Test "present"/"absent" for respiratory distress
    cat = scoring.CasualtyAssessmentTracker()
    report = copy.deepcopy(report_base)

    report[scoring.crk.INJURIES][scoring.crk.RESPIRATORY_DISTRESS] = False
    cat.track_report(report, 30)
    score, _ = scoring.score_hemorrhage_and_distress(cat, ground_truths, set())
    assert score == 4.0

    report[scoring.crk.INJURIES][scoring.crk.RESPIRATORY_DISTRESS] = True
    cat.track_report(report, 31)
    score, _ = scoring.score_hemorrhage_and_distress(cat, ground_truths, set())
    assert score == 4.0

    # Test "absent"|"normal" are right, "abnormal"|"nt" are wrong for motor alertness
    cat = scoring.CasualtyAssessmentTracker()
    report = copy.deepcopy(report_base)

    report[scoring.crk.INJURIES][scoring.crk.ALERTNESS][scoring.crk.MOTOR] = "nt"
    cat.track_report(report, 30)
    score, _ = scoring.score_alertness(cat, ground_truths, set())
    assert score == 0.0

    report[scoring.crk.INJURIES][scoring.crk.ALERTNESS][scoring.crk.MOTOR] = "abnormal"
    cat.track_report(report, 31)
    score, _ = scoring.score_alertness(cat, ground_truths, set())
    assert score == 0.0

    report[scoring.crk.INJURIES][scoring.crk.ALERTNESS][scoring.crk.MOTOR] = "normal"
    cat.track_report(report, 32)
    score, _ = scoring.score_alertness(cat, ground_truths, set())
    assert score == 1.0

    report[scoring.crk.INJURIES][scoring.crk.ALERTNESS][scoring.crk.VERBAL] = "absent"  # just to get us fully correct
    report[scoring.crk.INJURIES][scoring.crk.ALERTNESS][scoring.crk.MOTOR] = "absent"
    cat.track_report(report, 33)
    score, _ = scoring.score_alertness(cat, ground_truths, set())
    assert score == 2.0
