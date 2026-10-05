from living_map_writer.progress import StuckDetector
from living_map_writer.sortie import Sortie


def test_exploration_completes_after_consecutive_empty_checks():
    sortie = Sortie(budget_s=0, empty_ticks_to_complete=3)
    assert not sortie.no_frontier()
    sortie.frontier_found()
    assert not sortie.no_frontier() and not sortie.no_frontier()
    assert sortie.no_frontier()
    assert sortie.phase == "RETURNING" and "mapped" in sortie.reason
    sortie.arrived_home()
    assert sortie.phase == "HOME"


def test_budget_forces_return_and_repeated_failure_gives_up():
    sortie = Sortie(budget_s=10, max_return_attempts=2)
    sortie.spend(9.9)
    assert not sortie.budget_spent()
    sortie.spend(0.2)
    assert sortie.budget_spent()
    sortie.start_return("budget")
    assert not sortie.return_failed()
    assert sortie.return_failed()
    assert sortie.phase == "RETURN_FAILED"
    sortie.arrived_home()
    assert sortie.phase == "RETURN_FAILED"


def test_stuck_detector_needs_a_full_window_without_progress():
    detector = StuckDetector(window_s=10, min_progress_m=0.3)
    stuck = [detector.update(t * 0.5, 1.0 + 0.001 * t, 2.0, True) for t in range(21)]
    assert not any(stuck[:20]) and stuck[20]


def test_stuck_detector_ignores_progress_pauses_and_rotation():
    detector = StuckDetector(window_s=10, min_progress_m=0.3)
    assert not any(detector.update(t * 0.5, 0.05 * t, 0.0, True) for t in range(60))
    detector.reset()
    for t in range(15):
        detector.update(t * 0.5, 0.0, 0.0, True)
    assert not detector.update(8.0, 0.0, 0.0, False)  # paused: window restarts
    assert not detector.update(9.0, 0.0, 0.0, True)
