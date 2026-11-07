"""Goal-based navigation on a live map: the CAD gives the order of the
course and rough relative offsets; positions come from what the drone sees."""
from pathlib import Path


def resource(relative):
    """A file shipped with the package (config/, assets/): beside the source
    tree when run from it, else in the installed package's share directory."""
    here = Path(__file__).resolve().parents[2] / relative
    if here.exists():
        return here
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('drone_ref')) / relative
    except (ImportError, LookupError):
        return here
