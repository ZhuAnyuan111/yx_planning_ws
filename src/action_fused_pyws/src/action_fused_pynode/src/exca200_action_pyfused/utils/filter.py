from typing import Optional


def filter(input_data: float, previous_output: float, dt: float, max_rate_up: Optional[float], max_rate_down: Optional[float] = None) -> float:
    if dt <= 0.0:
        return input_data

    if max_rate_up is None:
        return input_data

    if max_rate_up < 0.0:
        raise ValueError("max_rate must be non-negative")

    max_delta_up = max_rate_up * dt
    delta = input_data - previous_output

    if delta >= max_delta_up:
        return previous_output + max_delta_up

    if delta <= 0.0:
        if max_rate_down is None or max_rate_down == 0.0:
            return input_data

        max_rate_down_signed = max_rate_down
        if max_rate_down_signed > 0.0:
            max_rate_down_signed = -max_rate_down_signed

        max_delta_down = max_rate_down_signed * dt
        if delta <= max_delta_down:
            return previous_output + max_delta_down

        return input_data

    return input_data
