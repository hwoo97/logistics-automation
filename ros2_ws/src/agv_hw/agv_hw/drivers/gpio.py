"""Pi 5 헤더 GPIO 칩(RP1) 열기 — 커널에 따라 gpiochip0 또는 gpiochip4 (03_setup/gpio_i2c 확인)."""


def open_header_chip(chip: int | None = None):
    """(lgpio 모듈, 핸들) 반환. chip을 주지 않으면 라벨에 'rp1'이 있는 칩을 찾는다."""
    import lgpio
    if chip is not None:
        return lgpio, lgpio.gpiochip_open(chip)
    for n in range(8):
        try:
            h = lgpio.gpiochip_open(n)
        except lgpio.error:
            continue
        label = str(lgpio.gpio_get_chip_info(h)[3]).lower()
        if "rp1" in label:
            return lgpio, h
        lgpio.gpiochip_close(h)
    raise RuntimeError("RP1 헤더 GPIO 칩을 찾지 못함 — 파라미터 gpio_chip 으로 지정")
