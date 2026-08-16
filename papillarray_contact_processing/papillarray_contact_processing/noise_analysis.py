"""分析 PapillArray 原始 CSV 的噪声频谱并比较低通截止频率。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import warnings
from collections.abc import Sequence
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from papillarray_contact_processing.low_pass_filter import FirstOrderLowPassFilter

DEFAULT_CUTOFFS_HZ = (5.0, 10.0, 20.0, 40.0, 60.0, 100.0)


@dataclass(slots=True, frozen=True)
class SamplingStats:
    """CSV 时间轴的采样质量统计。

    Attributes:
        sample_count: 有效样本数。
        duration_s: 首末样本覆盖时长 (s)。
        sampling_rate_hz: 由正时间间隔中位数估计的采样率 (Hz)。
        median_interval_us: 正时间间隔中位数 (us)。
        jitter_p99_us: 相对中位间隔的绝对偏差 99% 分位数 (us)。
        irregular_interval_ratio: 偏离中位间隔超过 20% 的比例。
        non_increasing_count: 时间戳重复或回退的次数。
    """

    sample_count: int
    duration_s: float
    sampling_rate_hz: float
    median_interval_us: float
    jitter_p99_us: float
    irregular_interval_ratio: float
    non_increasing_count: int


@dataclass(slots=True, frozen=True)
class FilterResult:
    """一个候选截止频率的离线滤波结果。

    Attributes:
        cutoff_hz: 候选截止频率 (Hz)。
        output_std: 滤波输出去趋势后的标准差。
        noise_reduction_pct: 相对原始标准差的降低比例。
        time_constant_ms: 对应连续一阶系统时间常数 (ms)。
    """

    cutoff_hz: float
    output_std: float
    noise_reduction_pct: float
    time_constant_ms: float


def load_channel_csv(
    csv_path: Path, sensor_index: int, field: str
) -> tuple[np.ndarray, np.ndarray]:
    """读取记录器 CSV 或实验室宽表 CSV 的一个传感器通道。

    Args:
        csv_path: 输入 CSV 路径。
        sensor_index: 传感器索引。
        field: 全局力或力矩字段名，如 ``gfz``。

    Returns:
        tuple[np.ndarray, np.ndarray]: 时间戳秒数组与信号数组。

    Raises:
        ValueError: CSV 缺列、数据不足或没有指定传感器样本时抛出。
    """
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV 缺少表头")
        names = {name.lower(): name for name in reader.fieldnames}
        time_column = names.get("tus") or names.get("t_us")
        if time_column is None:
            raise ValueError("CSV 缺少 tus 或 T_us 时间戳列")

        field_lower = field.lower()
        wide_name = f"s{sensor_index}_g_{field_lower[-2:]}"
        value_column = names.get(field_lower) or names.get(wide_name)
        if value_column is None:
            raise ValueError(
                f"CSV 缺少 {field} 或 S{sensor_index}_G_{field_lower[-2:].upper()} 列"
            )
        sensor_column = names.get("sensor_index")

        timestamps_us: list[float] = []
        values: list[float] = []
        for row in reader:
            if sensor_column is not None and int(row[sensor_column]) != sensor_index:
                continue
            timestamp_us = float(row[time_column])
            value = float(row[value_column])
            if math.isfinite(timestamp_us) and math.isfinite(value):
                timestamps_us.append(timestamp_us)
                values.append(value)

    if len(values) < 32:
        raise ValueError("有效数据点过少，至少需要 32 个样本")
    timestamps_s = np.asarray(timestamps_us, dtype=np.float64) * 1e-6
    signal = np.asarray(values, dtype=np.float64)
    return timestamps_s, signal


def discover_sensor_indices(csv_path: Path) -> list[int]:
    """从 CSV 中发现全部传感器索引。

    记录器 CSV 使用 ``sensor_index`` 列；实验室宽表 CSV 使用
    ``s{sensor_index}_g_*`` 列名。两者都没有时视为单个传感器 0。

    Args:
        csv_path: 输入 CSV 路径。

    Returns:
        list[int]: 升序去重后的传感器索引列表。

    Raises:
        ValueError: CSV 缺列或没有可识别的传感器时抛出。
    """
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV 缺少表头")
        names = {name.lower(): name for name in reader.fieldnames}
        sensor_column = names.get("sensor_index")
        indices: set[int] = set()
        if sensor_column is not None:
            for row in reader:
                try:
                    indices.add(int(row[sensor_column]))
                except (ValueError, TypeError):
                    continue
        else:
            for fieldname in reader.fieldnames:
                match = re.match(r"s(\d+)_g_", fieldname.lower())
                if match:
                    indices.add(int(match.group(1)))
        if not indices:
            indices.add(0)
    return sorted(indices)


def calculate_sampling_stats(timestamps_s: np.ndarray) -> SamplingStats:
    """计算实际采样率、时间戳抖动与异常间隔比例。

    Args:
        timestamps_s: 单调时间戳数组，单位秒。

    Returns:
        SamplingStats: 采样质量统计结果。

    Raises:
        ValueError: 没有正时间间隔时抛出。
    """
    intervals_s = np.diff(timestamps_s)
    positive = intervals_s[intervals_s > 0.0]
    if positive.size == 0:
        raise ValueError("时间戳没有正间隔，无法估计采样率")
    median_s = float(np.median(positive))
    jitter_s = np.abs(positive - median_s)
    return SamplingStats(
        sample_count=int(timestamps_s.size),
        duration_s=float(timestamps_s[-1] - timestamps_s[0]),
        sampling_rate_hz=1.0 / median_s,
        median_interval_us=median_s * 1e6,
        jitter_p99_us=float(np.percentile(jitter_s, 99.0) * 1e6),
        irregular_interval_ratio=float(np.mean(jitter_s > 0.2 * median_s)),
        non_increasing_count=int(np.sum(intervals_s <= 0.0)),
    )


def detrend_signal(signal: np.ndarray) -> np.ndarray:
    """去除信号的均值与线性趋势。

    Args:
        signal: 原始一维信号。

    Returns:
        np.ndarray: 去趋势后的信号。
    """
    indices = np.arange(signal.size, dtype=np.float64)
    slope, intercept = np.polyfit(indices, signal, 1)
    return signal - (slope * indices + intercept)


def welch_psd(
    signal: np.ndarray, sampling_rate_hz: float, nperseg: int
) -> tuple[np.ndarray, np.ndarray]:
    """仅用 NumPy 计算 Hann 窗、50% 重叠的单边 Welch PSD。

    Args:
        signal: 已去趋势的一维信号。
        sampling_rate_hz: 采样率 (Hz)。
        nperseg: 每个 Welch 分段的样本数。

    Returns:
        tuple[np.ndarray, np.ndarray]: 频率轴和功率谱密度。

    Raises:
        ValueError: 采样率或分段长度不合法时抛出。
    """
    if sampling_rate_hz <= 0.0 or nperseg < 8:
        raise ValueError("sampling_rate_hz 必须为正且 nperseg 至少为 8")
    segment_size = min(int(nperseg), int(signal.size))
    hop = max(1, segment_size // 2)
    window = np.hanning(segment_size)
    normalization = float(np.sum(window**2))
    segment_count = 1 + (signal.size - segment_size) // hop
    frequencies = np.fft.rfftfreq(segment_size, d=1.0 / sampling_rate_hz)
    psd = np.zeros(frequencies.size, dtype=np.float64)
    for index in range(segment_count):
        start = index * hop
        spectrum = np.fft.rfft(signal[start : start + segment_size] * window)
        psd += np.abs(spectrum) ** 2 / normalization
    psd /= float(segment_count * sampling_rate_hz)
    if segment_size % 2 == 0:
        psd[1:-1] *= 2.0
    else:
        psd[1:] *= 2.0
    return frequencies, psd


def apply_low_pass(
    signal: np.ndarray,
    timestamps_s: np.ndarray,
    cutoff_hz: float,
    reset_gap_s: float,
) -> np.ndarray:
    """使用在线节点同款算法对完整数组做离线低通。

    Args:
        signal: 原始信号数组。
        timestamps_s: 对应时间戳秒数组。
        cutoff_hz: 截止频率 (Hz)。
        reset_gap_s: 数据间隔重置阈值 (s)。

    Returns:
        np.ndarray: 滤波输出数组。
    """
    force_filter = FirstOrderLowPassFilter(cutoff_hz, reset_gap_s)
    return np.asarray(
        [
            force_filter.filter(float(sample), float(sample_time_s))
            for sample, sample_time_s in zip(signal, timestamps_s, strict=True)
        ],
        dtype=np.float64,
    )


def compare_cutoffs(
    signal: np.ndarray,
    timestamps_s: np.ndarray,
    cutoffs_hz: Sequence[float],
    reset_gap_s: float,
) -> tuple[list[FilterResult], dict[float, np.ndarray]]:
    """扫描候选截止频率并统计去噪比例。

    Args:
        signal: 原始信号数组。
        timestamps_s: 对应时间戳秒数组。
        cutoffs_hz: 候选截止频率序列。
        reset_gap_s: 数据间隔重置阈值 (s)。

    Returns:
        tuple[list[FilterResult], dict[float, np.ndarray]]: 统计和滤波波形。

    Raises:
        ValueError: 候选列表为空或包含非法截止频率时抛出。
    """
    if not cutoffs_hz or any(
        not math.isfinite(cutoff) or cutoff <= 0.0 for cutoff in cutoffs_hz
    ):
        raise ValueError("cutoffs_hz 必须包含有限正数")
    raw_std = float(np.std(detrend_signal(signal)))
    outputs: dict[float, np.ndarray] = {}
    results: list[FilterResult] = []
    for cutoff_hz in cutoffs_hz:
        cutoff = float(cutoff_hz)
        output = apply_low_pass(signal, timestamps_s, cutoff, reset_gap_s)
        output_std = float(np.std(detrend_signal(output)))
        reduction = 0.0 if raw_std <= 0.0 else (1.0 - output_std / raw_std) * 100.0
        outputs[cutoff] = output
        results.append(
            FilterResult(
                cutoff_hz=cutoff,
                output_std=output_std,
                noise_reduction_pct=reduction,
                time_constant_ms=1000.0 / (2.0 * math.pi * cutoff),
            )
        )
    return results, outputs


def render_plots(
    output_dir: Path,
    field: str,
    timestamps_s: np.ndarray,
    signal: np.ndarray,
    frequencies: np.ndarray,
    psd: np.ndarray,
    filtered_outputs: dict[float, np.ndarray],
    filter_results: Sequence[FilterResult],
    sampling_rate_hz: float,
    nperseg: int,
    window_s: float,
) -> tuple[Path, Path, Path, Path]:
    """输出 PSD、滤波波形对比、截止频率权衡与滤波 PSD 对比图。

    Args:
        output_dir: 图片输出目录。
        field: 被分析字段名。
        timestamps_s: 时间戳秒数组。
        signal: 原始信号。
        frequencies: PSD 频率轴。
        psd: 原始信号功率谱密度。
        filtered_outputs: 各候选截止频率的滤波波形。
        filter_results: 各候选截止频率的统计结果。
        sampling_rate_hz: 采样率 (Hz)。
        nperseg: Welch 分段长度。
        window_s: 波形对比图截取末尾时长 (s)；0 表示使用整段数据。

    Returns:
        tuple[Path, Path, Path, Path]: PSD 图、波形对比图、截止频率权衡图与
        滤波 PSD 对比图路径。
    """
    import matplotlib

    matplotlib.use("Agg")
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")
        import matplotlib.pyplot as plt
    try:
        import scienceplots  # noqa: F401  # 导入以注册 SciencePlots 样式

        plot_styles = ["science", "ieee", "no-latex", "grid"]
    except ImportError:
        # ros2 run 使用系统 Python，未安装 scienceplots 时回退到手动 IEEE 风格。
        plot_styles = []

    output_dir.mkdir(parents=True, exist_ok=True)
    psd_path = output_dir / "noise_psd.png"
    comparison_path = output_dir / "filter_comparison.png"
    tradeoff_path = output_dir / "cutoff_tradeoff.png"
    psd_comparison_path = output_dir / "psd_filter_comparison.png"
    elapsed_s = timestamps_s - timestamps_s[0]
    # 波形对比图默认截取末尾窗口；window_s <= 0 时使用整段数据。
    window_s = float(window_s)
    start_index = (
        0
        if window_s <= 0.0
        else int(np.searchsorted(elapsed_s, max(0.0, elapsed_s[-1] - window_s)))
    )
    time_view = elapsed_s[start_index:]
    raw_view = signal[start_index:]
    positive = frequencies > 0.0
    # 各截止频率滤波信号的 Welch PSD，与原始 PSD 同参数，频率轴一致。
    filtered_psds: dict[float, np.ndarray] = {}
    for cutoff_hz, output in filtered_outputs.items():
        _, filtered_psds[cutoff_hz] = welch_psd(
            detrend_signal(output), sampling_rate_hz, nperseg
        )

    with plt.style.context(plot_styles):
        plt.rcParams["font.family"] = [
            "Times New Roman",
            "Noto Serif CJK SC",
            "DejaVu Serif",
        ]
        plt.rcParams["font.serif"] = [
            "Times New Roman",
            "Noto Serif CJK SC",
            "DejaVu Serif",
        ]
        plt.rcParams["mathtext.fontset"] = "stix"
        plt.rcParams["axes.unicode_minus"] = False
        if not plot_styles:
            # 手动模拟 SciencePlots IEEE 风格：衬线、去顶/右边框、细网格。
            plt.rcParams["axes.spines.top"] = False
            plt.rcParams["axes.spines.right"] = False
            plt.rcParams["axes.grid"] = True
            plt.rcParams["grid.alpha"] = 0.25

        # PSD 图
        figure, axis = plt.subplots(figsize=(8.0, 4.5), layout="constrained")
        axis.loglog(frequencies[positive], psd[positive])
        axis.axvline(
            50.0, color="tab:red", linestyle="--", linewidth=1.0, label="50 Hz"
        )
        for cutoff_hz in filtered_outputs:
            axis.axvline(cutoff_hz, color="0.7", linestyle=":", linewidth=0.7)
        axis.set(
            title=f"{field} Welch PSD",
            xlabel="Frequency (Hz)",
            ylabel="PSD (unit²/Hz)",
        )
        axis.grid(True, which="both", alpha=0.25)
        axis.legend()
        figure.savefig(psd_path, dpi=300)
        plt.close(figure)

        # 波形对比图：每个截止频率一个子图，浅灰背景为原始信号
        n_cutoffs = len(filtered_outputs)
        total_panels = n_cutoffs
        ncols = total_panels if total_panels <= 4 else int(math.ceil(total_panels / 2))
        nrows = int(math.ceil(total_panels / ncols))
        figure, axes = plt.subplots(
            nrows,
            ncols,
            figsize=(3.4 * ncols, 3.0 * nrows),
            layout="constrained",
            sharex=True,
            squeeze=False,
        )
        flat = axes.ravel()
        for index, cutoff_hz in enumerate(filtered_outputs):
            axis = flat[index]
            axis.plot(time_view, raw_view, color="0.85", linewidth=0.7)
            axis.plot(
                time_view,
                filtered_outputs[cutoff_hz][start_index:],
                linewidth=1.0,
                label=f"{cutoff_hz:g} Hz",
            )
            if index % ncols == 0:
                axis.set_ylabel(field)
            axis.set(title=f"{cutoff_hz:g} Hz", xlabel="Time (s)")
            axis.grid(True, alpha=0.25)
            axis.legend(fontsize="small")
        for axis in flat[total_panels:]:
            axis.set_visible(False)
        window_label = "full recording" if window_s <= 0.0 else f"last {window_s:g} s"
        figure.suptitle(f"{field} filter comparison ({window_label})")
        figure.savefig(comparison_path, dpi=300)
        plt.close(figure)

        # 截止频率权衡图：左轴为降噪比例，右轴为一阶时间常数
        figure, axis = plt.subplots(figsize=(8.0, 4.5), layout="constrained")
        cutoffs_hz = [result.cutoff_hz for result in filter_results]
        reductions_pct = [result.noise_reduction_pct for result in filter_results]
        time_constants_ms = [result.time_constant_ms for result in filter_results]
        axis.semilogx(
            cutoffs_hz,
            reductions_pct,
            color="tab:blue",
            marker="o",
            linewidth=1.2,
            label="noise reduction",
        )
        axis.set(
            title=f"{field} cutoff tradeoff",
            xlabel="Cutoff (Hz)",
            ylabel="Noise reduction (%)",
        )
        axis.grid(True, which="both", alpha=0.25)
        axis.set_ylim(0.0, 100.0)
        twin = axis.twinx()
        twin.semilogx(
            cutoffs_hz,
            time_constants_ms,
            color="tab:red",
            marker="s",
            linestyle="--",
            linewidth=1.0,
            label="time constant",
        )
        twin.set_ylabel("Time constant (ms)")
        lines_1, labels_1 = axis.get_legend_handles_labels()
        lines_2, labels_2 = twin.get_legend_handles_labels()
        axis.legend(
            lines_1 + lines_2,
            labels_1 + labels_2,
            loc="center right",
            fontsize="small",
        )
        figure.savefig(tradeoff_path, dpi=300)
        plt.close(figure)

        # 滤波 PSD 对比图：每个截止频率一个子图，
        # 浅灰为原始 PSD，实线为实测滤波 PSD，虚线为理论一阶响应 |H(f)|² 叠加
        n_cutoffs = len(filtered_outputs)
        total_panels = n_cutoffs
        ncols = total_panels if total_panels <= 4 else int(math.ceil(total_panels / 2))
        nrows = int(math.ceil(total_panels / ncols))
        figure, axes = plt.subplots(
            nrows,
            ncols,
            figsize=(3.4 * ncols, 3.0 * nrows),
            layout="constrained",
            squeeze=False,
        )
        flat = axes.ravel()
        for index, cutoff_hz in enumerate(filtered_outputs):
            axis = flat[index]
            axis.loglog(
                frequencies[positive], psd[positive], color="0.75", linewidth=0.7
            )
            axis.loglog(
                frequencies[positive],
                filtered_psds[cutoff_hz][positive],
                linewidth=1.0,
                label=f"{cutoff_hz:g} Hz",
            )
            theory = psd / (1.0 + (frequencies / cutoff_hz) ** 2.0)
            axis.loglog(
                frequencies[positive],
                theory[positive],
                linestyle="--",
                linewidth=1.0,
                label=f"{cutoff_hz:g} Hz theory",
            )
            axis.axvline(cutoff_hz, color="0.5", linestyle=":", linewidth=0.7)
            axis.set(title=f"{cutoff_hz:g} Hz", xlabel="Frequency (Hz)")
            if index % ncols == 0:
                axis.set_ylabel("PSD (unit²/Hz)")
            axis.grid(True, which="both", alpha=0.25)
            axis.legend(fontsize="small")
        for axis in flat[total_panels:]:
            axis.set_visible(False)
        figure.suptitle(f"{field} PSD comparison (raw vs filtered)")
        figure.savefig(psd_comparison_path, dpi=300)
        plt.close(figure)
    return psd_path, comparison_path, tradeoff_path, psd_comparison_path


def run_analysis(
    csv_path: Path,
    output_dir: Path,
    sensor_index: int,
    field: str,
    cutoffs_hz: Sequence[float],
    nperseg: int,
    window_s: float = 5.0,
) -> dict[str, object]:
    """执行完整噪声分析并写出 JSON、CSV 与图片结果。

    Args:
        csv_path: 原始 CSV 路径。
        output_dir: 结果目录。
        sensor_index: 传感器索引。
        field: 分析字段。
        cutoffs_hz: 候选截止频率。
        nperseg: Welch 分段长度。
        window_s: 波形对比图截取末尾时长 (s)；0 表示使用整段数据。

    Returns:
        dict[str, object]: 可序列化的分析摘要。
    """
    timestamps_s, signal = load_channel_csv(csv_path, sensor_index, field)
    sampling = calculate_sampling_stats(timestamps_s)
    detrended = detrend_signal(signal)
    frequencies, psd = welch_psd(detrended, sampling.sampling_rate_hz, nperseg)
    reset_gap_s = max(0.1, 10.0 / sampling.sampling_rate_hz)
    filter_results, filtered_outputs = compare_cutoffs(
        signal, timestamps_s, cutoffs_hz, reset_gap_s
    )

    output_dir.mkdir(parents=True, exist_ok=True)
    (
        psd_path,
        comparison_path,
        tradeoff_path,
        psd_comparison_path,
    ) = render_plots(
        output_dir,
        field,
        timestamps_s,
        signal,
        frequencies,
        psd,
        filtered_outputs,
        filter_results,
        sampling.sampling_rate_hz,
        nperseg,
        window_s,
    )
    raw_stats = {
        "mean": float(np.mean(signal)),
        "std": float(np.std(signal)),
        "detrended_std": float(np.std(detrended)),
        "rms": float(np.sqrt(np.mean(signal**2))),
        "peak_to_peak": float(np.ptp(signal)),
        "p01": float(np.percentile(signal, 1.0)),
        "p99": float(np.percentile(signal, 99.0)),
    }
    summary: dict[str, object] = {
        "source": str(csv_path),
        "sensor_index": sensor_index,
        "field": field,
        "sampling": asdict(sampling),
        "raw": raw_stats,
        "filters": [asdict(result) for result in filter_results],
        "plots": [
            str(psd_path),
            str(comparison_path),
            str(tradeoff_path),
            str(psd_comparison_path),
        ],
    }
    with (output_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    with (output_dir / "cutoff_comparison.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=FilterResult.__dataclass_fields__)
        writer.writeheader()
        writer.writerows(asdict(result) for result in filter_results)
    return summary


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析离线分析命令行参数。

    Args:
        argv: 待解析参数；``None`` 表示使用进程命令行。

    Returns:
        argparse.Namespace: 解析后的参数。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path, help="原始传感器 CSV 路径")
    parser.add_argument(
        "--output-dir", type=Path, default=Path("noise_analysis_result")
    )
    parser.add_argument(
        "--sensor",
        type=int,
        nargs="*",
        default=None,
        help="传感器索引列表（如 0 1）；不指定时自动分析 CSV 中全部传感器",
    )
    parser.add_argument(
        "--field",
        choices=("gfx", "gfy", "gfz", "gtx", "gty", "gtz"),
        default="gfz",
        help="待分析的全局力/力矩字段",
    )
    parser.add_argument(
        "--cutoffs",
        type=float,
        nargs="+",
        default=DEFAULT_CUTOFFS_HZ,
        help="候选低通截止频率列表 (Hz)",
    )
    parser.add_argument("--nperseg", type=int, default=1024, help="Welch 分段长度")
    parser.add_argument(
        "--window-s",
        type=float,
        default=5.0,
        help="波形对比图截取末尾时长 (秒)；0 表示使用整段数据",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    """运行命令行噪声分析并打印关键结果。

    Args:
        argv: 可选命令行参数序列。
    """
    args = parse_args(argv)
    sensor_indices = args.sensor or discover_sensor_indices(args.csv_path)
    try:
        for sensor_index in sensor_indices:
            sensor_output = args.output_dir / f"sensor_{sensor_index}"
            summary = run_analysis(
                args.csv_path,
                sensor_output,
                sensor_index,
                args.field,
                args.cutoffs,
                args.nperseg,
                args.window_s,
            )
            sampling = summary["sampling"]
            raw = summary["raw"]
            print(f"[sensor {sensor_index}]")
            print(
                f"  样本={sampling['sample_count']}，实际采样率={sampling['sampling_rate_hz']:.3f} Hz，"
                f"异常间隔={sampling['irregular_interval_ratio']:.2%}"
            )
            print(
                f"  {args.field}: 均值={raw['mean']:.6g}，"
                f"去趋势标准差={raw['detrended_std']:.6g}，峰峰值={raw['peak_to_peak']:.6g}"
            )
            for result in summary["filters"]:
                print(
                    f"  fc={result['cutoff_hz']:g} Hz: 标准差={result['output_std']:.6g}，"
                    f"降低={result['noise_reduction_pct']:.1f}%"
                )
            print(f"  结果目录: {sensor_output}")
    except (OSError, ValueError) as error:
        raise SystemExit(f"分析失败: {error}") from error


if __name__ == "__main__":
    main()
