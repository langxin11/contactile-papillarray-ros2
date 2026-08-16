"""离线噪声分析核心算法单元测试。"""

import csv
from pathlib import Path

import numpy as np
import pytest
from papillarray_contact_processing.noise_analysis import (
    calculate_sampling_stats,
    compare_cutoffs,
    detrend_signal,
    load_channel_csv,
    run_analysis,
    welch_psd,
)


def test_sampling_stats_detects_500_hz_data() -> None:
    """2 ms 均匀时间戳应被识别为 500 Hz 且没有异常间隔。"""
    timestamps_s = np.arange(1000, dtype=np.float64) * 0.002

    stats = calculate_sampling_stats(timestamps_s)

    assert stats.sampling_rate_hz == pytest.approx(500.0)
    assert stats.irregular_interval_ratio == pytest.approx(0.0)
    assert stats.non_increasing_count == 0


def test_welch_psd_finds_sine_peak() -> None:
    """Welch PSD 的最大非直流谱峰应接近输入正弦频率。"""
    sampling_rate_hz = 500.0
    timestamps_s = np.arange(4096, dtype=np.float64) / sampling_rate_hz
    signal = np.sin(2.0 * np.pi * 50.0 * timestamps_s)

    frequencies, psd = welch_psd(signal, sampling_rate_hz, 1024)

    peak_hz = frequencies[1:][int(np.argmax(psd[1:]))]
    assert peak_hz == pytest.approx(50.0, abs=0.6)


def test_lower_cutoff_reduces_white_noise_more() -> None:
    """同一白噪声下较低截止频率应得到更小的输出标准差。"""
    generator = np.random.default_rng(7)
    timestamps_s = np.arange(5000, dtype=np.float64) / 500.0
    signal = generator.normal(size=timestamps_s.size)

    results, _ = compare_cutoffs(signal, timestamps_s, [5.0, 100.0], 0.1)

    assert results[0].output_std < results[1].output_std
    assert results[0].noise_reduction_pct > results[1].noise_reduction_pct


def test_recorder_csv_selects_requested_sensor(tmp_path: Path) -> None:
    """长表 CSV 应只加载指定 sensor_index 的原始通道。"""
    csv_path = tmp_path / "raw.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["tus", "sensor_index", "gfz"])
        writer.writeheader()
        for index in range(40):
            writer.writerow({"tus": index * 2000, "sensor_index": 0, "gfz": index})
            writer.writerow(
                {"tus": index * 2000, "sensor_index": 1, "gfz": 100 + index}
            )

    timestamps_s, signal = load_channel_csv(csv_path, 1, "gfz")

    assert timestamps_s.size == 40
    assert signal[0] == pytest.approx(100.0)
    assert signal[-1] == pytest.approx(139.0)


def test_detrend_removes_linear_component() -> None:
    """线性输入去趋势后应接近零。"""
    signal = 3.0 * np.arange(100, dtype=np.float64) + 7.0

    detrended = detrend_signal(signal)

    assert np.max(np.abs(detrended)) < 1e-10


def test_run_analysis_writes_complete_result_set(tmp_path: Path) -> None:
    """完整分析应生成统计 JSON、截止频率 CSV 和四张结果图。"""
    csv_path = tmp_path / "raw.csv"
    generator = np.random.default_rng(11)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["tus", "sensor_index", "gfz"])
        writer.writeheader()
        for index, noise in enumerate(generator.normal(scale=0.02, size=2048)):
            writer.writerow(
                {"tus": index * 2000, "sensor_index": 0, "gfz": 0.5 + noise}
            )
    output_dir = tmp_path / "result"

    summary = run_analysis(csv_path, output_dir, 0, "gfz", [10.0, 20.0], 512)

    assert summary["sampling"]["sampling_rate_hz"] == pytest.approx(500.0)
    assert (output_dir / "summary.json").is_file()
    assert (output_dir / "cutoff_comparison.csv").is_file()
    assert (output_dir / "noise_psd.png").is_file()
    assert (output_dir / "filter_comparison.png").is_file()
    assert (output_dir / "cutoff_tradeoff.png").is_file()
    assert (output_dir / "psd_filter_comparison.png").is_file()
