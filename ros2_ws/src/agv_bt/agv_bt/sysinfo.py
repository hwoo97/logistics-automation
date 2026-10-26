"""Pi 상태(온도·CPU·메모리) — /sys, /proc 읽기. 파싱은 순수 함수로 분리해 시험한다."""

TEMP_PATH = "/sys/class/thermal/thermal_zone0/temp"


def parse_temp(text: str) -> float:
    return int(text.strip()) / 1000.0              # 밀리도 → ℃


def parse_cpu_times(stat_text: str) -> tuple[int, int]:
    """/proc/stat 첫 줄 → (전체, 유휴) 누적 시간."""
    fields = [int(v) for v in stat_text.splitlines()[0].split()[1:]]
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)     # idle + iowait
    return sum(fields), idle


def parse_mem_percent(meminfo_text: str) -> float:
    info = {}
    for line in meminfo_text.splitlines():
        key, _, rest = line.partition(":")
        info[key] = int(rest.split()[0]) if rest.split() else 0
    total, avail = info["MemTotal"], info.get("MemAvailable", info.get("MemFree", 0))
    return (total - avail) / total * 100.0


class SysMonitor:
    def __init__(self, read=None):
        self.read = read or (lambda path: open(path, encoding="ascii").read())
        self._prev = None

    def sample(self) -> tuple[float, float, float]:
        """(온도 ℃, CPU %, 메모리 %). 첫 호출의 CPU는 0."""
        try:
            temp = parse_temp(self.read(TEMP_PATH))
        except (OSError, ValueError):
            temp = 0.0
        total, idle = parse_cpu_times(self.read("/proc/stat"))
        cpu = 0.0
        if self._prev:
            dt, di = total - self._prev[0], idle - self._prev[1]
            cpu = (1 - di / dt) * 100.0 if dt > 0 else 0.0
        self._prev = (total, idle)
        return temp, cpu, parse_mem_percent(self.read("/proc/meminfo"))
