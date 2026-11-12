import os
import sys

# Run Qt headless and make the top-level modules importable.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

SAMPLE_SLED_INFO = {
    "serialNumberBoard": "TEST00001",
    "partNumberBoard": "00000000000",
    "versionBoard": "N",
    "serialNumberLepton": "000000",
    "versionLepton": "0.3.47",
    "leptonQR": "A0000000",
    "versionRosebudFactoryESW": "1.0.8",
    "versionRosebudOperationalESW": "1.0.11",
    "versionRosebudUpdaterESW": "1.0.11",
    "versionRosebudAPI": "rosebud-maintenance.2c9f18c",
    "gitRevision": "rosebud-maintenance.2c9f18c",
    "operatingMode": "operational",
}
