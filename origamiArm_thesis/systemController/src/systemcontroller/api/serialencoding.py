from dataclasses import dataclass
from typing import Final


@dataclass(frozen=True, kw_only=True)
class MsgTypeEncoding:
    """Fixed data defining encoding for message types."""

    enq: Final[str]
    ack: Final[str]
    angles: Final[str]
    errorcode: Final[str]
    sendpos: Final[str]


@dataclass(frozen=True, kw_only=True)
class MsgEncoding:
    """Fixed data defining message encoding."""

    start: Final[str]
    end: Final[str]
    angle_sep: Final[str]
    data_sep: Final[str]
    type: MsgTypeEncoding


MSG: Final[MsgEncoding] = MsgEncoding(
    start="$",
    end="%",
    angle_sep="&",
    data_sep=",",
    type=MsgTypeEncoding(enq="?", ack="#", angles=">", errorcode="!", sendpos="@"),
)

ERRORCODES: Final[dict[int, str]] = {1: "message receive error", 2: "limit switch hit"}
