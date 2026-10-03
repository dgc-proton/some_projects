from dataclasses import dataclass
from typing import Any, Final


@dataclass(frozen=False, kw_only=True)
class FrameCoords:
    """Coordinates on camera in pixels."""

    x_px: int
    y_px: int


@dataclass(frozen=False, kw_only=True)
class LocationData:
    """Data for a specific visual location."""

    code: Final[int | None] = None
    description: Final[str]
    mm_per_px: Final[float]
    coords_vis_px_x: int
    coords_vis_px_y: int
    been_detected: bool = False


    @property
    def coords_phys_mm_x(self) -> int:
        """Translates the visual coordinates to physical coordinate system.

        @property gives slight performance penalty but better encapsulation.
        Visual coordinates are in pixels, with origin being top left of image,
        physical coordinates are in mm with origin at top right of image.
                         =====>
         pixels      x            x         mm
           +--------->            <---------+
           |                                |
           |                                |
           |                                |
         y v                                v y

        """
        return int(self.coords_vis_px_x * -self.mm_per_px)

    @property
    def coords_phys_mm_y(self) -> int:
        """Translates the visual coordinates to physical coordinate system.

        @property gives slight performance penalty but better encapsulation.
        Visual coordinates are in pixels, with origin being top left of image,
        physical coordinates are in mm with origin at top right of image.
                         =====>
         pixels      x            x         mm
           +--------->            <---------+
           |                                |
           |                                |
           |                                |
         y v                                v y

        """
        return int(self.coords_vis_px_y * self.mm_per_px)
