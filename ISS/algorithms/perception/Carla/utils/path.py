from pathlib import Path

# Project Root Path
ROOT_PATH = Path(__file__).parent.parent.as_posix()

RAW_DATA_PATH = "{}/{}".format(ROOT_PATH, 'raw_data')
DATASET_PATH = "{}/{}".format(ROOT_PATH, 'dataset')