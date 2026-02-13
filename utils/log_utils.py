import time
from pathlib import Path


def rename_daily_log_file_name(log_file: str | Path) -> Path:
    log_file_stem = Path(log_file).stem
    log_file_suffix = Path(log_file).suffix
    date_str = time.strftime("%Y%m%d")
    return Path(log_file).with_name(f"{log_file_stem}_{date_str}{log_file_suffix}")
