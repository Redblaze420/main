from typing import *
CONFIRMED: object
CANCELLED: object
INFO: object


# rust/src/ui/model_tr/layout.rs
def disable_animation(disable: bool) -> None:
    """Disable animations, debug builds only."""


# rust/src/ui/model_tr/layout.rs
def confirm_action(
    *,
    title: str,
    action: str | None,
    description: str | None,
    verb: str = "CONFIRM",
    verb_cancel: str | None = None,
    hold: bool = False,
    hold_danger: bool = False,  # unused on TR
    reverse: bool = False,
) -> object:
    """Confirm action."""


# rust/src/ui/model_tr/layout.rs
def confirm_blob(
    *,
    title: str,
    data: str | bytes,
    description: str | None,
    extra: str | None,
    verb_cancel: str | None = None,
    hold: bool = False,
) -> object:
    """Confirm byte sequence data."""


# rust/src/ui/model_tr/layout.rs
def confirm_properties(
    *,
    title: str,
    items: Iterable[Tuple[str | None, str | None, bool]],
    hold: bool = False,
) -> object:
    """Confirm list of key-value pairs. The third component in the tuple should be True if
    the value is to be rendered as binary with monospace font, False otherwise.
    This only concerns the text style, you need to decode the value to UTF-8 in python."""


# rust/src/ui/model_tr/layout.rs
def confirm_output_r(
    *,
    address: str,
    amount: str,
    address_title: str,
    amount_title: str,
) -> object:
    """Confirm output. Specific for model R."""


# rust/src/ui/model_tr/layout.rs
def confirm_total_r(
    *,
    total_amount: str,
    fee_amount: str,
    fee_rate_amount: str | None = None,
    total_label: str,
    fee_label: str,
) -> object:
    """Confirm summary of a transaction. Specific for model R."""


# rust/src/ui/model_tr/layout.rs
def show_receive_address(
    *,
    title: str,
    address: str,
    address_qr: str,
    account: str,
    derivation_path: str,
    case_sensitive: bool,
) -> object:
    """Show receive address together with QR code and details about it."""


# rust/src/ui/model_tr/layout.rs
def show_info(
    *,
    title: str,
    description: str = "",
    time_ms: int = 0,
) -> object:
    """Info modal."""


# rust/src/ui/model_tr/layout.rs
def confirm_fido(
    *,
    app_name: str,
    accounts: list[str | None],
) -> int | object:
    """FIDO confirmation.
    Returns page index in case of confirmation and CANCELLED otherwise.
    """


# rust/src/ui/model_tr/layout.rs
def tutorial() -> object:
    """Show user how to interact with the device."""


# rust/src/ui/model_tr/layout.rs
def request_pin(
    *,
    prompt: str,
    subprompt: str | None = None,
    allow_cancel: bool | None = None,
) -> str | object:
    """Request pin on device."""


# rust/src/ui/model_tr/layout.rs
def request_number(
    *,
    title: str,
    count: int,
    min_count: int,
    max_count: int,
) -> object:
   """Number input with + and - buttons, description, and info button."""


# rust/src/ui/model_tr/layout.rs
def show_checklist(
    *,
    title: str,
    items: Iterable[str],
    active: int,
    button: str,
) -> object:
   """Checklist of backup steps. Active index is highlighted, previous items have check
   mark next to them."""


# rust/src/ui/model_tr/layout.rs
def show_share_words(
    *,
    title: str,
    share_words: Iterable[str],
) -> None:
    """Shows a backup seed."""


# rust/src/ui/model_tr/layout.rs
def select_word(
    *,
    title: str,
    words: Iterable[str],
) -> int:
   """Select mnemonic word from three possibilities - seed check after backup. The
   iterable must be of exact size. Returns index in range `0..3`."""


# rust/src/ui/model_tr/layout.rs
def select_word_count(
    *,
    dry_run: bool,
) -> str:  # TODO: make it return int
   """Select mnemonic word count from (12, 18, 20, 24, 33)."""


# rust/src/ui/model_tr/layout.rs
def request_bip39(
    *,
    prompt: str,
) -> str:
    """Get recovery word for BIP39."""


# rust/src/ui/model_tr/layout.rs
def request_slip39(
    *,
    prompt: str,
) -> str:
   """SLIP39 word input keyboard."""


# rust/src/ui/model_tr/layout.rs
def request_passphrase(
    *,
    prompt: str,
    max_len: int,
) -> str:
    """Get passphrase."""


# rust/src/ui/model_tr/layout.rs
def show_progress(
    *,
    title: str,
    indeterminate: bool = False,
    description: str | None = None,
) -> object:
   """Show progress loader. Please note that the number of lines reserved on screen for
   description is determined at construction time. If you want multiline descriptions
   make sure the initial description has at least that amount of lines."""


# rust/src/ui/model_tr/layout.rs
def show_homescreen(
    *,
    label: str,
    hold: bool,
    notification: str | None,
    notification_level: int = 0,
    skip_first_paint: bool,
) -> CANCELLED:
    """Idle homescreen."""


# rust/src/ui/model_tr/layout.rs
def show_lockscreen(
    *,
    label: str,
    bootscreen: bool,
    skip_first_paint: bool,
) -> CANCELLED:
    """Homescreen for locked device."""


# rust/src/ui/model_tr/layout.rs
def show_busyscreen(
    *,
    title: str,
    description: str,
    time_ms: int,
    skip_first_paint: bool,
) -> CANCELLED:
    """Homescreen used for indicating coinjoin in progress."""
CONFIRMED: object
CANCELLED: object
INFO: object


# rust/src/ui/model_tt/layout.rs
def disable_animation(disable: bool) -> None:
    """Disable animations, debug builds only."""


# rust/src/ui/model_tt/layout.rs
def jpeg_info(data: bytes) -> (width: int, height: int, mcu_height: int):
    """Get JPEG image dimensions."""


# rust/src/ui/model_tt/layout.rs
def jpeg_test(data: bytes) -> bool:
    """Test JPEG image."""


# rust/src/ui/model_tt/layout.rs
def confirm_action(
    *,
    title: str,
    action: str | None,
    description: str | None,
    verb: str = "CONFIRM",
    verb_cancel: str | None = None,
    hold: bool = False,
    hold_danger: bool = False,
    reverse: bool = False,
) -> object:
    """Confirm action."""


# rust/src/ui/model_tt/layout.rs
def confirm_homescreen(
    *,
    title: str,
    image: bytes,
) -> object:
    """Confirm homescreen."""


# rust/src/ui/model_tt/layout.rs
def confirm_blob(
    *,
    title: str,
    data: str | bytes,
    description: str | None,
    extra: str | None,
    verb_cancel: str | None = None,
    hold: bool = False,
) -> object:
    """Confirm byte sequence data."""


# rust/src/ui/model_tt/layout.rs
def confirm_properties(
    *,
    title: str,
    items: list[tuple[str | None, str | bytes | None, bool]],
    hold: bool = False,
) -> object:
    """Confirm list of key-value pairs. The third component in the tuple should be True if
    the value is to be rendered as binary with monospace font, False otherwise."""


# rust/src/ui/model_tt/layout.rs
def confirm_reset_device(
    *,
    title: str,
    prompt: str,
) -> object:
    """Confirm TOS before device setup."""


# rust/src/ui/model_tt/layout.rs
def show_qr(
    *,
    title: str,
    address: str,
    verb_cancel: str,
    case_sensitive: bool,
) -> object:
    """Show QR code."""


# rust/src/ui/model_tt/layout.rs
def confirm_value(
    *,
    title: str,
    description: str,
    value: str,
    verb: str | None = None,
    hold: bool = False,
) -> object:
    """Confirm value. Merge of confirm_total and confirm_output."""


# rust/src/ui/model_tt/layout.rs
def confirm_joint_total(
    *,
    spending_amount: str,
    total_amount: str,
) -> object:
    """Confirm total if there are external inputs."""


# rust/src/ui/model_tt/layout.rs
def confirm_modify_output(
    *,
    address: str,
    sign: int,
    amount_change: str,
    amount_new: str,
) -> object:
    """Decrease or increase amount for given address."""


# rust/src/ui/model_tt/layout.rs
def confirm_modify_fee(
    *,
    sign: int,
    user_fee_change: str,
    total_fee_new: str,
) -> object:
    """Decrease or increase transaction fee."""


# rust/src/ui/model_tt/layout.rs
def confirm_fido(
    *,
    title: str,
    app_name: str,
    icon_name: str | None,
    accounts: list[str | None],
) -> int | object:
    """FIDO confirmation.
    Returns page index in case of confirmation and CANCELLED otherwise.
    """


# rust/src/ui/model_tt/layout.rs
def show_error(
    *,
    title: str,
    button: str = "CONTINUE",
    description: str = "",
    allow_cancel: bool = False,
    time_ms: int = 0,
) -> object:
    """Error modal. No buttons shown when `button` is empty string."""


# rust/src/ui/model_tt/layout.rs
def show_warning(
    *,
    title: str,
    button: str = "CONTINUE",
    description: str = "",
    allow_cancel: bool = False,
    time_ms: int = 0,
) -> object:
    """Warning modal. No buttons shown when `button` is empty string."""


# rust/src/ui/model_tt/layout.rs
def show_success(
    *,
    title: str,
    button: str = "CONTINUE",
    description: str = "",
    allow_cancel: bool = False,
    time_ms: int = 0,
) -> object:
    """Success modal. No buttons shown when `button` is empty string."""


# rust/src/ui/model_tt/layout.rs
def show_info(
    *,
    title: str,
    button: str = "CONTINUE",
    description: str = "",
    allow_cancel: bool = False,
    time_ms: int = 0,
) -> object:
    """Info modal. No buttons shown when `button` is empty string."""


# rust/src/ui/model_tt/layout.rs
def show_simple(
    *,
    title: str | None,
    description: str,
    button: str,
) -> object:
    """Simple dialog with text and one button."""


# rust/src/ui/model_tt/layout.rs
def confirm_with_info(
    *,
    title: str,
    button: str,
    info_button: str,
    items: Iterable[Tuple[int, str]],
) -> object:
    """Confirm given items but with third button. Always single page
    without scrolling."""


# rust/src/ui/model_tt/layout.rs
def confirm_more(
    *,
    title: str,
    button: str,
    items: Iterable[Tuple[int, str]],
) -> object:
    """Confirm long content with the possibility to go back from any page.
    Meant to be used with confirm_with_info."""


# rust/src/ui/model_tt/layout.rs
def confirm_coinjoin(
    *,
    max_rounds: str,
    max_feerate: str,
) -> object:
    """Confirm coinjoin authorization."""


# rust/src/ui/model_tt/layout.rs
def request_pin(
    *,
    prompt: str,
    subprompt: str,
    allow_cancel: bool = True,
    wrong_pin: bool = False,
) -> str | object:
    """Request pin on device."""


# rust/src/ui/model_tt/layout.rs
def request_passphrase(
    *,
    prompt: str,
    max_len: int,
) -> str | object:
   """Passphrase input keyboard."""


# rust/src/ui/model_tt/layout.rs
def request_bip39(
    *,
    prompt: str,
) -> str:
   """BIP39 word input keyboard."""


# rust/src/ui/model_tt/layout.rs
def request_slip39(
    *,
    prompt: str,
) -> str:
   """SLIP39 word input keyboard."""


# rust/src/ui/model_tt/layout.rs
def select_word(
    *,
    title: str,
    description: str,
    words: Iterable[str],
) -> int:
   """Select mnemonic word from three possibilities - seed check after backup. The
   iterable must be of exact size. Returns index in range `0..3`."""


# rust/src/ui/model_tt/layout.rs
def show_share_words(
    *,
    title: str,
    pages: Iterable[str],
) -> object:
   """Show mnemonic for backup. Expects the words pre-divided into individual pages."""


# rust/src/ui/model_tt/layout.rs
def request_number(
    *,
    title: str,
    count: int,
    min_count: int,
    max_count: int,
    description: Callable[[int], str] | None = None,
) -> object:
   """Number input with + and - buttons, description, and info button."""


# rust/src/ui/model_tt/layout.rs
def show_checklist(
    *,
    title: str,
    items: Iterable[str],
    active: int,
    button: str,
) -> object:
   """Checklist of backup steps. Active index is highlighted, previous items have check
   mark next to them."""


# rust/src/ui/model_tt/layout.rs
def confirm_recovery(
    *,
    title: str,
    description: str,
    button: str,
    dry_run: bool,
    info_button: bool,
) -> object:
   """Device recovery homescreen."""


# rust/src/ui/model_tt/layout.rs
def select_word_count(
    *,
    dry_run: bool,
) -> int | CANCELLED:
   """Select mnemonic word count from (12, 18, 20, 24, 33)."""


# rust/src/ui/model_tt/layout.rs
def show_group_share_success(
    *,
    lines: Iterable[str]
) -> int:
   """Shown after successfully finishing a group."""


# rust/src/ui/model_tt/layout.rs
def show_remaining_shares(
    *,
    pages: Iterable[tuple[str, str]],
) -> int:
   """Shows SLIP39 state after info button is pressed on `confirm_recovery`."""


# rust/src/ui/model_tt/layout.rs
def show_progress(
    *,
    title: str,
    indeterminate: bool = False,
    description: str | None = None,
) -> object:
   """Show progress loader. Please note that the number of lines reserved on screen for
   description is determined at construction time. If you want multiline descriptions
   make sure the initial description has at least that amount of lines."""


# rust/src/ui/model_tt/layout.rs
def show_homescreen(
    *,
    label: str,
    hold: bool,
    notification: str | None,
    notification_level: int = 0,
    skip_first_paint: bool,
) -> CANCELLED:
    """Idle homescreen."""


# rust/src/ui/model_tt/layout.rs
def show_lockscreen(
    *,
    label: str,
    bootscreen: bool,
    skip_first_paint: bool,
) -> CANCELLED:
    """Homescreen for locked device."""


# rust/src/ui/model_tt/layout.rs
def show_busyscreen(
    *,
    title: str,
    description: str,
    time_ms: int,
    skip_first_paint: bool,
) -> CANCELLED:
    """Homescreen used for indicating coinjoin in progress."""
