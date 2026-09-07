from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Union

import matplotlib.pyplot as plt
import numpy as np

ArrayLike = Union[float, np.ndarray]


# ============================================================
# 1. Data structures
# ============================================================

@dataclass(frozen=True)
class PacejkaCoefficients:
    """Simplified Pacejka Magic Formula coefficients."""

    B: float  # Stiffness factor
    C: float  # Shape factor
    D: float  # Peak friction coefficient, approximately F_peak / Fz
    E: float  # Curvature factor


@dataclass(frozen=True)
class CharacteristicPoint:
    """One characteristic point on the positive-slip branch."""

    x: float
    y: float
    index: int


@dataclass(frozen=True)
class TireCharacteristicPoints:
    """Three characteristic points used to divide the curve into four sections."""

    linear_end: CharacteristicPoint
    peak: CharacteristicPoint
    saturation_start: CharacteristicPoint

    # Exact tangent slope at the origin for the normalized Pacejka curve.
    initial_tangent_slope: float

    # Slope of the actual first linear segment connecting (0, 0) to P1.
    first_segment_slope: float

    # Errors at the selected section boundaries.
    linear_error: float
    saturation_error: float


@dataclass(frozen=True)
class Line:
    """y = slope * x + intercept"""

    slope: float
    intercept: float

    def evaluate(self, x: ArrayLike) -> ArrayLike:
        result = self.slope * np.asarray(x, dtype=float) + self.intercept
        return float(result) if np.ndim(x) == 0 else result


@dataclass(frozen=True)
class FourSegmentLinearModel:
    """
    Four-section model for the positive-slip force magnitude.

    Section 1: (0, 0) -> P1
    Section 2: P1 -> P2
    Section 3: P2 -> P3
    Section 4: constant y = P3.y
    """

    segment_1: Line
    segment_2: Line
    segment_3: Line
    segment_4: Line

    x1: float
    x2: float
    x3: float


# ============================================================
# 2. Generic game-car tire coefficients
# ============================================================

LONGITUDINAL = PacejkaCoefficients(
    B=18.656,
    C=1.6048,
    D=1.160,
    E=0.8088,
)

LATERAL = PacejkaCoefficients(
    B=17.125,
    C=1.5240,
    D=0.689,
    E=0.2088,
)


# ============================================================
# 3. Pacejka Magic Formula
# ============================================================

def pacejka_normalized(
    slip: ArrayLike,
    coefficients: PacejkaCoefficients,
) -> ArrayLike:
    """
    Return normalized tire force F / Fz.

    Parameters
    ----------
    slip:
        Longitudinal: slip ratio, e.g. 10% = 0.10
        Lateral: slip angle in radians
    coefficients:
        Simplified Pacejka B, C, D, E coefficients

    Notes
    -----
    This function is odd-symmetric:
        positive slip -> positive normalized force
        negative slip -> negative normalized force
    """
    x = np.asarray(slip, dtype=float)

    bs = coefficients.B * x
    inner = bs - coefficients.E * (bs - np.arctan(bs))

    result = coefficients.D * np.sin(
        coefficients.C * np.arctan(inner)
    )

    return float(result) if np.ndim(slip) == 0 else result


def longitudinal_force(
    slip_ratio: ArrayLike,
    vertical_load: float,
) -> ArrayLike:
    """Return longitudinal force Fx [N]."""
    fz = max(float(vertical_load), 0.0)
    result = fz * np.asarray(
        pacejka_normalized(slip_ratio, LONGITUDINAL),
        dtype=float,
    )
    return float(result) if np.ndim(slip_ratio) == 0 else result


def lateral_force(
    slip_angle_rad: ArrayLike,
    vertical_load: float,
) -> ArrayLike:
    """
    Return lateral force Fy [N].

    The minus sign assumes tire force opposes the slip-angle direction.
    Remove it if the coordinate convention in the game is opposite.
    """
    fz = max(float(vertical_load), 0.0)
    result = -fz * np.asarray(
        pacejka_normalized(slip_angle_rad, LATERAL),
        dtype=float,
    )
    return float(result) if np.ndim(slip_angle_rad) == 0 else result


# ============================================================
# 4. Characteristic-point extraction
# ============================================================

def find_characteristic_points(
    slip: np.ndarray,
    normalized_force: np.ndarray,
    coefficients: PacejkaCoefficients,
    linear_tolerance: float = 0.05,
    saturation_tolerance: float = 0.02,
) -> TireCharacteristicPoints:
    """
    Find P1, P2, P3 on the positive-slip branch.

    P1: End of the initial linear section
        The final sample before the error from the origin tangent
        exceeds `linear_tolerance * peak_force`.

    P2: Peak point
        The sample with maximum normalized force magnitude.

    P3: Start of the constant saturation section
        The earliest sample after P2 such that replacing every later
        value by force[P3] produces a maximum tail error no larger than
        `saturation_tolerance * peak_force`.

    Parameters
    ----------
    slip:
        Strictly increasing 1-D array starting at 0.
    normalized_force:
        Normalized force F/Fz sampled at `slip`.
    coefficients:
        Pacejka coefficients used to calculate the exact origin slope B*C*D.
    linear_tolerance:
        Allowed initial-line error as a fraction of peak force.
    saturation_tolerance:
        Allowed constant-tail error as a fraction of peak force.
    """
    x = np.asarray(slip, dtype=float)
    y = np.abs(np.asarray(normalized_force, dtype=float))

    if x.ndim != 1 or y.ndim != 1:
        raise ValueError("slip and normalized_force must be 1-D arrays.")
    if len(x) != len(y):
        raise ValueError("slip and normalized_force must have equal lengths.")
    if len(x) < 4:
        raise ValueError("At least four samples are required.")
    if not np.isclose(x[0], 0.0):
        raise ValueError("slip must start at 0.")
    if np.any(np.diff(x) <= 0.0):
        raise ValueError("slip must be strictly increasing.")
    if not 0.0 < linear_tolerance < 1.0:
        raise ValueError("linear_tolerance must be between 0 and 1.")
    if not 0.0 < saturation_tolerance < 1.0:
        raise ValueError("saturation_tolerance must be between 0 and 1.")

    # P2: peak
    peak_index = int(np.argmax(y))
    peak_force = float(y[peak_index])

    if peak_index < 2:
        raise ValueError(
            "The peak occurs too close to the origin. "
            "Increase the sampling range or inspect the coefficients."
        )
    if peak_force <= 0.0:
        raise ValueError("Peak force must be positive.")

    peak_point = CharacteristicPoint(
        x=float(x[peak_index]),
        y=peak_force,
        index=peak_index,
    )

    # P1: end of initial linear region, based on exact origin tangent.
    initial_tangent_slope = (
        coefficients.B * coefficients.C * coefficients.D
    )
    tangent_y = initial_tangent_slope * x
    tangent_error = np.abs(y - tangent_y)
    allowed_linear_error = linear_tolerance * peak_force

    # Ignore index 0 and search only before the peak.
    invalid = np.flatnonzero(
        tangent_error[1: peak_index + 1] > allowed_linear_error
    )

    if len(invalid) == 0:
        linear_end_index = peak_index - 1
    else:
        # +1 restores the sliced-array offset.
        first_invalid_index = int(invalid[0] + 1)
        linear_end_index = max(1, first_invalid_index - 1)

    linear_end_point = CharacteristicPoint(
        x=float(x[linear_end_index]),
        y=float(y[linear_end_index]),
        index=linear_end_index,
    )

    first_segment_slope = (
        linear_end_point.y / linear_end_point.x
    )

    # P3: earliest point after P2 whose constant tail stays within tolerance.
    suffix_min = np.minimum.accumulate(y[::-1])[::-1]
    suffix_max = np.maximum.accumulate(y[::-1])[::-1]

    tail_error_if_constant = np.maximum(
        np.abs(suffix_max - y),
        np.abs(y - suffix_min),
    )
    allowed_saturation_error = saturation_tolerance * peak_force

    search_start = peak_index + 1
    valid = np.flatnonzero(
        tail_error_if_constant[search_start:]
        <= allowed_saturation_error
    )

    if len(valid) == 0:
        saturation_start_index = len(x) - 1
    else:
        saturation_start_index = search_start + int(valid[0])

    saturation_start_point = CharacteristicPoint(
        x=float(x[saturation_start_index]),
        y=float(y[saturation_start_index]),
        index=saturation_start_index,
    )

    return TireCharacteristicPoints(
        linear_end=linear_end_point,
        peak=peak_point,
        saturation_start=saturation_start_point,
        initial_tangent_slope=float(initial_tangent_slope),
        first_segment_slope=float(first_segment_slope),
        linear_error=float(tangent_error[linear_end_index]),
        saturation_error=float(
            tail_error_if_constant[saturation_start_index]
        ),
    )


def calculate_longitudinal_characteristic_points(
    maximum_slip_ratio: float = 1.0,
    sample_count: int = 100_001,
    linear_tolerance: float = 0.02,
    saturation_tolerance: float = 0.02,
) -> tuple[np.ndarray, np.ndarray, TireCharacteristicPoints]:
    """Sample the positive longitudinal branch and find P1, P2, P3."""
    if maximum_slip_ratio <= 0.0:
        raise ValueError("maximum_slip_ratio must be positive.")

    slip_ratio = np.linspace(
        0.0,
        maximum_slip_ratio,
        sample_count,
    )
    normalized_fx = np.asarray(
        pacejka_normalized(slip_ratio, LONGITUDINAL),
        dtype=float,
    )

    points = find_characteristic_points(
        slip=slip_ratio,
        normalized_force=normalized_fx,
        coefficients=LONGITUDINAL,
        linear_tolerance=linear_tolerance,
        saturation_tolerance=saturation_tolerance,
    )

    return slip_ratio, normalized_fx, points


def calculate_lateral_characteristic_points(
    maximum_angle_deg: float = 30.0,
    sample_count: int = 100_001,
    linear_tolerance: float = 0.02,
    saturation_tolerance: float = 0.02,
) -> tuple[np.ndarray, np.ndarray, TireCharacteristicPoints]:
    """Sample the positive lateral branch and find P1, P2, P3."""
    if maximum_angle_deg <= 0.0:
        raise ValueError("maximum_angle_deg must be positive.")

    slip_angle_rad = np.linspace(
        0.0,
        np.deg2rad(maximum_angle_deg),
        sample_count,
    )
    normalized_fy_magnitude = np.asarray(
        pacejka_normalized(slip_angle_rad, LATERAL),
        dtype=float,
    )

    points = find_characteristic_points(
        slip=slip_angle_rad,
        normalized_force=normalized_fy_magnitude,
        coefficients=LATERAL,
        linear_tolerance=linear_tolerance,
        saturation_tolerance=saturation_tolerance,
    )

    return slip_angle_rad, normalized_fy_magnitude, points


# ============================================================
# 5. Four-segment linear approximation
# ============================================================

def line_through_points(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
) -> Line:
    """Return the line passing through two points."""
    if np.isclose(x1, x0):
        raise ValueError("The two x coordinates must be different.")

    slope = (y1 - y0) / (x1 - x0)
    intercept = y0 - slope * x0

    return Line(
        slope=float(slope),
        intercept=float(intercept),
    )


def build_four_segment_linear_model(
    points: TireCharacteristicPoints,
) -> FourSegmentLinearModel:
    """Build the four-section positive-branch approximation."""
    p1 = points.linear_end
    p2 = points.peak
    p3 = points.saturation_start

    if not (0.0 < p1.x < p2.x < p3.x):
        raise ValueError(
            "Characteristic points must satisfy 0 < P1.x < P2.x < P3.x."
        )

    return FourSegmentLinearModel(
        segment_1=line_through_points(
            0.0, 0.0,
            p1.x, p1.y,
        ),
        segment_2=line_through_points(
            p1.x, p1.y,
            p2.x, p2.y,
        ),
        segment_3=line_through_points(
            p2.x, p2.y,
            p3.x, p3.y,
        ),
        segment_4=Line(
            slope=0.0,
            intercept=p3.y,
        ),
        x1=p1.x,
        x2=p2.x,
        x3=p3.x,
    )


def evaluate_four_segment_model(
    slip: ArrayLike,
    model: FourSegmentLinearModel,
) -> ArrayLike:
    """
    Evaluate the four-section approximation.

    The positive-branch magnitude is mirrored as an odd function.
    """
    x = np.asarray(slip, dtype=float)
    abs_x = np.abs(x)

    magnitude = np.select(
        [
            abs_x <= model.x1,
            (abs_x > model.x1) & (abs_x <= model.x2),
            (abs_x > model.x2) & (abs_x <= model.x3),
            abs_x > model.x3,
        ],
        [
            model.segment_1.slope * abs_x
            + model.segment_1.intercept,

            model.segment_2.slope * abs_x
            + model.segment_2.intercept,

            model.segment_3.slope * abs_x
            + model.segment_3.intercept,

            model.segment_4.slope * abs_x
            + model.segment_4.intercept,
        ],
    )

    result = np.sign(x) * magnitude

    return float(result) if np.ndim(slip) == 0 else result


def longitudinal_force_linear(
    slip_ratio: ArrayLike,
    vertical_load: float,
    model: FourSegmentLinearModel,
) -> ArrayLike:
    """Return longitudinal force from the four-section approximation."""
    fz = max(float(vertical_load), 0.0)
    result = fz * np.asarray(
        evaluate_four_segment_model(slip_ratio, model),
        dtype=float,
    )
    return float(result) if np.ndim(slip_ratio) == 0 else result


def lateral_force_linear(
    slip_angle_rad: ArrayLike,
    vertical_load: float,
    model: FourSegmentLinearModel,
) -> ArrayLike:
    """Return lateral force from the four-section approximation."""
    fz = max(float(vertical_load), 0.0)
    result = -fz * np.asarray(
        evaluate_four_segment_model(slip_angle_rad, model),
        dtype=float,
    )
    return float(result) if np.ndim(slip_angle_rad) == 0 else result


# ============================================================
# 6. Error analysis
# ============================================================

def calculate_error_metrics(
    reference: np.ndarray,
    approximation: np.ndarray,
) -> dict[str, float]:
    """Return basic approximation-error metrics."""
    ref = np.asarray(reference, dtype=float)
    approx = np.asarray(approximation, dtype=float)

    if ref.shape != approx.shape:
        raise ValueError("reference and approximation must have equal shapes.")

    error = approx - ref
    absolute_error = np.abs(error)
    peak_reference = float(np.max(np.abs(ref)))

    return {
        "max_abs_error": float(np.max(absolute_error)),
        "mean_abs_error": float(np.mean(absolute_error)),
        "rmse": float(np.sqrt(np.mean(error ** 2))),
        "max_error_percent_of_peak": (
            float(np.max(absolute_error) / peak_reference * 100.0)
            if peak_reference > 0.0
            else 0.0
        ),
    }


# ============================================================
# 7. Reporting
# ============================================================

def print_characteristic_points(
    name: str,
    points: TireCharacteristicPoints,
    x_unit: str,
    x_scale: float = 1.0,
) -> None:
    """Print characteristic points in a readable form."""
    print(f"\n{name}")
    print("-" * len(name))

    print(
        f"P1 linear end       : "
        f"x={points.linear_end.x * x_scale:.6f} {x_unit}, "
        f"F/Fz={points.linear_end.y:.6f}"
    )
    print(
        f"P2 peak             : "
        f"x={points.peak.x * x_scale:.6f} {x_unit}, "
        f"F/Fz={points.peak.y:.6f}"
    )
    print(
        f"P3 saturation start : "
        f"x={points.saturation_start.x * x_scale:.6f} {x_unit}, "
        f"F/Fz={points.saturation_start.y:.6f}"
    )
    print(
        f"Initial tangent slope B*C*D : "
        f"{points.initial_tangent_slope:.6f}"
    )
    print(
        f"Section-1 line slope        : "
        f"{points.first_segment_slope:.6f}"
    )
    print(
        f"Selected P1 tangent error   : "
        f"{points.linear_error:.6f}"
    )
    print(
        f"Selected P3 tail error      : "
        f"{points.saturation_error:.6f}"
    )


def print_linear_model(
    name: str,
    model: FourSegmentLinearModel,
) -> None:
    """Print y = m*x + b for all four sections."""
    print(f"\n{name} four-section linear model")
    print("-" * (len(name) + 27))

    sections = [
        ("Section 1", 0.0, model.x1, model.segment_1),
        ("Section 2", model.x1, model.x2, model.segment_2),
        ("Section 3", model.x2, model.x3, model.segment_3),
        ("Section 4", model.x3, np.inf, model.segment_4),
    ]

    for section_name, start, end, line in sections:
        end_text = f"{end:.8f}" if np.isfinite(end) else "infinity"
        print(
            f"{section_name}: "
            f"{start:.8f} <= |x| <= {end_text}, "
            f"y = {line.slope:.8f} * |x| "
            f"+ {line.intercept:.8f}"
        )


# ============================================================
# 8. Plotting
# ============================================================

def add_characteristic_points_to_plot(
    points: TireCharacteristicPoints,
    x_transform,
) -> None:
    """Add P1, P2, P3 markers and labels to the active plot."""
    names = ["P1", "P2", "P3"]
    point_list = [
        points.linear_end,
        points.peak,
        points.saturation_start,
    ]

    plot_x = [x_transform(point.x) for point in point_list]
    plot_y = [point.y for point in point_list]

    plt.scatter(plot_x, plot_y, zorder=3)

    for name, x_value, y_value in zip(names, plot_x, plot_y):
        plt.annotate(
            name,
            xy=(x_value, y_value),
            xytext=(7, 7),
            textcoords="offset points",
        )


def plot_longitudinal_comparison(
    slip_ratio: np.ndarray,
    original: np.ndarray,
    model: FourSegmentLinearModel,
    points: TireCharacteristicPoints,
) -> None:
    """Plot original and four-section longitudinal normalized curves."""
    approximate = np.asarray(
        evaluate_four_segment_model(slip_ratio, model),
        dtype=float,
    )

    plt.figure(figsize=(10, 6))
    plt.plot(
        slip_ratio * 100.0,
        original,
        label="Pacejka",
    )
    plt.plot(
        slip_ratio * 100.0,
        approximate,
        linestyle="--",
        label="4-section linear",
    )

    add_characteristic_points_to_plot(
        points=points,
        x_transform=lambda x: x * 100.0,
    )

    plt.xlabel("Slip ratio κ [%]")
    plt.ylabel("Normalized longitudinal force Fx/Fz")
    plt.title("Longitudinal Pacejka and 4-section approximation")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


def plot_lateral_comparison(
    slip_angle_rad: np.ndarray,
    original: np.ndarray,
    model: FourSegmentLinearModel,
    points: TireCharacteristicPoints,
) -> None:
    """Plot original and four-section lateral normalized curves."""
    approximate = np.asarray(
        evaluate_four_segment_model(slip_angle_rad, model),
        dtype=float,
    )

    plt.figure(figsize=(10, 6))
    plt.plot(
        np.rad2deg(slip_angle_rad),
        original,
        label="Pacejka",
    )
    plt.plot(
        np.rad2deg(slip_angle_rad),
        approximate,
        linestyle="--",
        label="4-section linear",
    )

    add_characteristic_points_to_plot(
        points=points,
        x_transform=np.rad2deg,
    )

    plt.xlabel("Slip angle α [deg]")
    plt.ylabel("Normalized lateral-force magnitude |Fy|/Fz")
    plt.title("Lateral Pacejka and 4-section approximation")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


def plot_error_curve(
    x: np.ndarray,
    original: np.ndarray,
    approximate: np.ndarray,
    title: str,
    x_label: str,
) -> None:
    """Plot signed approximation error."""
    error = approximate - original

    plt.figure(figsize=(10, 5))
    plt.plot(x, error)
    plt.axhline(0.0, linewidth=0.8)
    plt.xlabel(x_label)
    plt.ylabel("Approximation error in F/Fz")
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()


# ============================================================
# 9. CSV export
# ============================================================

def export_model_data(
    output_directory: str | Path,
    longitudinal_slip: np.ndarray,
    longitudinal_original: np.ndarray,
    longitudinal_approximation: np.ndarray,
    longitudinal_points: TireCharacteristicPoints,
    lateral_angle_rad: np.ndarray,
    lateral_original: np.ndarray,
    lateral_approximation: np.ndarray,
    lateral_points: TireCharacteristicPoints,
) -> None:
    """Save sampled functions and characteristic points as CSV files."""
    output_path = Path(output_directory)
    output_path.mkdir(parents=True, exist_ok=True)

    longitudinal_table = np.column_stack(
        (
            longitudinal_slip,
            longitudinal_original,
            longitudinal_approximation,
            longitudinal_approximation - longitudinal_original,
        )
    )
    np.savetxt(
        output_path / "longitudinal_model.csv",
        longitudinal_table,
        delimiter=",",
        header=(
            "slip_ratio,"
            "pacejka_fx_over_fz,"
            "linear_fx_over_fz,"
            "linear_error"
        ),
        comments="",
    )

    lateral_table = np.column_stack(
        (
            lateral_angle_rad,
            np.rad2deg(lateral_angle_rad),
            lateral_original,
            lateral_approximation,
            lateral_approximation - lateral_original,
        )
    )
    np.savetxt(
        output_path / "lateral_model.csv",
        lateral_table,
        delimiter=",",
        header=(
            "slip_angle_rad,"
            "slip_angle_deg,"
            "pacejka_fy_magnitude_over_fz,"
            "linear_fy_magnitude_over_fz,"
            "linear_error"
        ),
        comments="",
    )

    longitudinal_point_table = np.array(
        [
            [
                1,
                longitudinal_points.linear_end.x,
                longitudinal_points.linear_end.y,
            ],
            [
                2,
                longitudinal_points.peak.x,
                longitudinal_points.peak.y,
            ],
            [
                3,
                longitudinal_points.saturation_start.x,
                longitudinal_points.saturation_start.y,
            ],
        ],
        dtype=float,
    )
    np.savetxt(
        output_path / "longitudinal_characteristic_points.csv",
        longitudinal_point_table,
        delimiter=",",
        header="point_number,slip_ratio,normalized_force",
        comments="",
    )

    lateral_point_table = np.array(
        [
            [
                1,
                lateral_points.linear_end.x,
                np.rad2deg(lateral_points.linear_end.x),
                lateral_points.linear_end.y,
            ],
            [
                2,
                lateral_points.peak.x,
                np.rad2deg(lateral_points.peak.x),
                lateral_points.peak.y,
            ],
            [
                3,
                lateral_points.saturation_start.x,
                np.rad2deg(lateral_points.saturation_start.x),
                lateral_points.saturation_start.y,
            ],
        ],
        dtype=float,
    )
    np.savetxt(
        output_path / "lateral_characteristic_points.csv",
        lateral_point_table,
        delimiter=",",
        header=(
            "point_number,"
            "slip_angle_rad,"
            "slip_angle_deg,"
            "normalized_force"
        ),
        comments="",
    )

    print(f"\nCSV files saved to: {output_path.resolve()}")


# ============================================================
# 10. Main
# ============================================================

def main() -> None:
    # --------------------------------------------------------
    # Settings
    # --------------------------------------------------------
    vertical_load = 4000.0  # N, example load on one tire

    longitudinal_max_slip = 1.0   # 100%
    lateral_max_angle_deg = 30.0

    sample_count = 100_001

    # Both tolerances are relative to the peak normalized force.
    linear_tolerance = 0.02
    saturation_tolerance = 0.02

    # --------------------------------------------------------
    # Longitudinal model
    # --------------------------------------------------------
    (
        longitudinal_slip,
        longitudinal_original,
        longitudinal_points,
    ) = calculate_longitudinal_characteristic_points(
        maximum_slip_ratio=longitudinal_max_slip,
        sample_count=sample_count,
        linear_tolerance=linear_tolerance,
        saturation_tolerance=saturation_tolerance,
    )

    longitudinal_model = build_four_segment_linear_model(
        longitudinal_points
    )
    longitudinal_approximation = np.asarray(
        evaluate_four_segment_model(
            longitudinal_slip,
            longitudinal_model,
        ),
        dtype=float,
    )

    # --------------------------------------------------------
    # Lateral model
    # --------------------------------------------------------
    (
        lateral_angle_rad,
        lateral_original,
        lateral_points,
    ) = calculate_lateral_characteristic_points(
        maximum_angle_deg=lateral_max_angle_deg,
        sample_count=sample_count,
        linear_tolerance=linear_tolerance,
        saturation_tolerance=saturation_tolerance,
    )

    lateral_model = build_four_segment_linear_model(
        lateral_points
    )
    lateral_approximation = np.asarray(
        evaluate_four_segment_model(
            lateral_angle_rad,
            lateral_model,
        ),
        dtype=float,
    )

    # --------------------------------------------------------
    # Console output
    # --------------------------------------------------------
    print_characteristic_points(
        name="Longitudinal characteristic points",
        points=longitudinal_points,
        x_unit="%",
        x_scale=100.0,
    )
    print_linear_model(
        name="Longitudinal",
        model=longitudinal_model,
    )

    print_characteristic_points(
        name="Lateral characteristic points",
        points=lateral_points,
        x_unit="deg",
        x_scale=180.0 / np.pi,
    )
    print_linear_model(
        name="Lateral",
        model=lateral_model,
    )

    longitudinal_error = calculate_error_metrics(
        longitudinal_original,
        longitudinal_approximation,
    )
    lateral_error = calculate_error_metrics(
        lateral_original,
        lateral_approximation,
    )

    print("\nLongitudinal approximation error")
    print("--------------------------------")
    for key, value in longitudinal_error.items():
        print(f"{key}: {value:.8f}")

    print("\nLateral approximation error")
    print("---------------------------")
    for key, value in lateral_error.items():
        print(f"{key}: {value:.8f}")

    # Example force evaluations.
    example_kappa = 0.12
    example_alpha_rad = np.deg2rad(8.0)

    fx_pacejka = longitudinal_force(
        example_kappa,
        vertical_load,
    )
    fx_linear = longitudinal_force_linear(
        example_kappa,
        vertical_load,
        longitudinal_model,
    )

    fy_pacejka = lateral_force(
        example_alpha_rad,
        vertical_load,
    )
    fy_linear = lateral_force_linear(
        example_alpha_rad,
        vertical_load,
        lateral_model,
    )

    print("\nExample function values")
    print("-----------------------")
    print(
        f"Longitudinal at κ={example_kappa:.3f}: "
        f"Pacejka={fx_pacejka:.2f} N, "
        f"linear={fx_linear:.2f} N"
    )
    print(
        f"Lateral at α={np.rad2deg(example_alpha_rad):.2f} deg: "
        f"Pacejka={fy_pacejka:.2f} N, "
        f"linear={fy_linear:.2f} N"
    )

    # --------------------------------------------------------
    # CSV export
    # --------------------------------------------------------
    export_model_data(
        output_directory="pacejka_output",
        longitudinal_slip=longitudinal_slip,
        longitudinal_original=longitudinal_original,
        longitudinal_approximation=longitudinal_approximation,
        longitudinal_points=longitudinal_points,
        lateral_angle_rad=lateral_angle_rad,
        lateral_original=lateral_original,
        lateral_approximation=lateral_approximation,
        lateral_points=lateral_points,
    )

    # --------------------------------------------------------
    # Plots
    # --------------------------------------------------------
    plot_longitudinal_comparison(
        slip_ratio=longitudinal_slip,
        original=longitudinal_original,
        model=longitudinal_model,
        points=longitudinal_points,
    )

    plot_lateral_comparison(
        slip_angle_rad=lateral_angle_rad,
        original=lateral_original,
        model=lateral_model,
        points=lateral_points,
    )

    plot_error_curve(
        x=longitudinal_slip * 100.0,
        original=longitudinal_original,
        approximate=longitudinal_approximation,
        title="Longitudinal 4-section approximation error",
        x_label="Slip ratio κ [%]",
    )

    plot_error_curve(
        x=np.rad2deg(lateral_angle_rad),
        original=lateral_original,
        approximate=lateral_approximation,
        title="Lateral 4-section approximation error",
        x_label="Slip angle α [deg]",
    )

    plt.show()


if __name__ == "__main__":
    main()
