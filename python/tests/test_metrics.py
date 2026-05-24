from __future__ import annotations

import numpy as np
import pytest

from polycal import interval_score, mpiw, picp


def test_picp_all_covered() -> None:
    intervals = np.tile(np.array([[-1.0, 1.0]] * 6), (4, 1, 1))
    ground_truths = np.array(
        [
            [0.0, 0.5, -0.5, 1.0, -1.0, 0.25],
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6],
            [1.0, -1.0, 0.0, 0.75, -0.75, 0.1],
        ],
        dtype=float,
    )

    np.testing.assert_allclose(picp(intervals, ground_truths), np.ones(6))


def test_picp_none_covered() -> None:
    intervals = np.tile(np.array([[0.0, 1.0]] * 6), (3, 1, 1))
    ground_truths = np.array(
        [
            [-1.0, 2.0, -2.0, 3.0, -3.0, 4.0],
            [-1.5, 2.5, -2.5, 3.5, -3.5, 4.5],
            [-2.0, 3.0, -3.0, 4.0, -4.0, 5.0],
        ],
        dtype=float,
    )

    np.testing.assert_allclose(picp(intervals, ground_truths), np.zeros(6))


def test_picp_single_sample() -> None:
    intervals = np.array(
        [
            [
                [0.0, 1.0],
                [0.0, 1.0],
                [0.0, 1.0],
                [0.0, 1.0],
                [0.0, 1.0],
                [0.0, 1.0],
            ]
        ],
        dtype=float,
    )
    ground_truths = np.array([[0.0, 0.5, 1.0, -0.1, 1.1, 2.0]], dtype=float)

    np.testing.assert_allclose(picp(intervals, ground_truths), np.array([1, 1, 1, 0, 0, 0]))


def test_mpiw_constant_width() -> None:
    intervals = np.tile(np.array([[-1.0, 1.0]] * 6), (5, 1, 1))

    np.testing.assert_allclose(mpiw(intervals), 2.0 * np.ones(6))


def test_mpiw_single_sample() -> None:
    intervals = np.array(
        [
            [
                [0.0, 1.0],
                [0.0, 2.0],
                [1.0, 4.0],
                [-2.0, 2.0],
                [-3.0, 2.0],
                [10.0, 16.0],
            ]
        ],
        dtype=float,
    )

    np.testing.assert_allclose(mpiw(intervals), np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))


def test_interval_score_no_penalty() -> None:
    intervals = np.array(
        [
            [[0.0, 2.0], [0.0, 4.0], [0.0, 6.0], [0.0, 8.0], [0.0, 10.0], [0.0, 12.0]],
            [[1.0, 3.0], [1.0, 5.0], [1.0, 7.0], [1.0, 9.0], [1.0, 11.0], [1.0, 13.0]],
        ],
        dtype=float,
    )
    ground_truths = np.array(
        [
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            [2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
        ],
        dtype=float,
    )

    np.testing.assert_allclose(interval_score(intervals, ground_truths, alpha=0.1), mpiw(intervals))


def test_interval_score_penalty() -> None:
    intervals = np.tile(np.array([[0.0, 1.0]] * 6), (2, 1, 1))
    ground_truths = np.array([[-0.5] * 6, [-1.0] * 6], dtype=float)

    expected = np.array([1.0 + 20.0 * 0.5, 1.0 + 20.0 * 1.0]).mean()
    np.testing.assert_allclose(interval_score(intervals, ground_truths, alpha=0.1), np.full(6, expected))


def test_interval_score_single_sample() -> None:
    intervals = np.array(
        [
            [
                [0.0, 1.0],
                [0.0, 2.0],
                [0.0, 3.0],
                [0.0, 4.0],
                [0.0, 5.0],
                [0.0, 6.0],
            ]
        ],
        dtype=float,
    )
    ground_truths = np.array([[0.5, -1.0, 4.0, 2.0, -0.5, 7.5]], dtype=float)

    expected = np.array([1.0, 22.0, 23.0, 4.0, 15.0, 36.0])
    np.testing.assert_allclose(interval_score(intervals, ground_truths, alpha=0.1), expected, atol=1e-6)


def test_gaussian_coverage_sanity() -> None:
    rng = np.random.default_rng(0)
    ground_truths = rng.normal(0.0, 1.0, size=(2000, 6))
    intervals = np.tile(np.array([[-1.645, 1.645]] * 6), (2000, 1, 1))

    coverage = picp(intervals, ground_truths)
    assert np.all((coverage >= 0.88) & (coverage <= 0.92))


def test_shapes_enforced() -> None:
    intervals = np.zeros((2, 6, 2))
    ground_truths = np.zeros((2, 6))

    with pytest.raises(ValueError, match="intervals must have shape"):
        picp(np.zeros((2, 6)), ground_truths)
    with pytest.raises(ValueError, match="ground_truths must have shape"):
        picp(intervals, np.zeros((3, 6)))
    with pytest.raises(ValueError, match="at least one sample"):
        mpiw(np.zeros((0, 6, 2)))
    with pytest.raises(ValueError, match="upper bounds"):
        mpiw(np.array([[[1.0, 0.0]] * 6]))
    with pytest.raises(ValueError, match="alpha"):
        interval_score(intervals, ground_truths, alpha=1.0)
