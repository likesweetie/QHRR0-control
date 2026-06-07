from __future__ import annotations


BUTTON_FIELDS = (
    "a_button",
    "b_button",
    "x_button",
    "y_button",
    "lb_button",
    "rb_button",
    "back_button",
    "start_button",
    "guide_button",
    "l3_button",
    "r3_button",
)


def buttons_to_mask(buttons: dict[str, bool]) -> int:
    mask = 0
    for index, name in enumerate(BUTTON_FIELDS):
        if bool(buttons.get(name, False)):
            mask |= 1 << index
    return mask


def mask_to_buttons(mask: int) -> dict[str, bool]:
    return {
        name: bool(int(mask) & (1 << index))
        for index, name in enumerate(BUTTON_FIELDS)
    }
