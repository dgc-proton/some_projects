try:  # Python 3.11 and newer has this in standard lib
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib
import pathlib
import sys
from importlib import resources
from typing import Any


def create_default_config() -> bool:
    """Creates the default configuration file. Retuns True if successful."""
    destination: pathlib.PosixPath = resources.files("systemcontroller").joinpath(
        "config.toml"
    )
    source: pathlib.PosixPath = resources.files("systemcontroller.utils").joinpath(
        "default_cfg.toml"
    )
    if destination.exists():
        print(
            "Error: create_default_config() called, but config appears to already exist"
        )
        return False
    source.copy(target=destination, preserve_metadata=True)
    print("Default config file created.")
    print("Please update config.toml then re-run this program. Exiting...")
    sys.exit()


def load_config() -> dict[str, Any]:
    """Returns the configuration as a dictionary."""
    config_dict: dict[str, Any] | None = None
    attempt: int = 1
    while not config_dict:
        print(f"Loading config file attempt {attempt}...")
        attempt += 1
        try:
            with resources.open_text("systemcontroller", "config.toml") as t:
                config_dict: dict[str, Any] = tomllib.loads(t.read())
        except FileNotFoundError:
            print("Config file not found, creating default config file")
            create_default_config()
        if attempt > 3:
            msg: str = "Error loading config, exiting"
            raise FileNotFoundError(msg)
    return config_dict
