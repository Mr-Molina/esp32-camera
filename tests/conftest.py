"""pytest configuration shared by the script tests.

Puts ``scripts/`` on ``sys.path`` before any test module is imported, so
the tests can ``import ota_deploy`` at the top of the file like any other
module. Doing this here instead of inside each test file avoids E402
(imports after code), which different ruff versions enable differently —
the pre-commit hook pins one and the lint step installs another — and which
previously needed ``noqa`` comments that the newer version then reported as
unused.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
