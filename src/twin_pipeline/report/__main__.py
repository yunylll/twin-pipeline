"""Write the report page for an existing run: ``python -m twin_pipeline.report <run folder> [--out page.html]``.

For runs that stopped before the report stage (the runner stops at a failed stage) or to refresh the
page after a resume. It reads run.json and the result.json files; it does not write a result.json.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import write_for_run


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m twin_pipeline.report", description="Write the HTML report of a pipeline run.")
    parser.add_argument("run_dir", type=Path, help="runs/<robot>/<run>")
    parser.add_argument("--out", type=Path, help="where to write the page (default: <run>/report/report.html)")
    parser.add_argument("--title", help="page title (default: report.title of twin.yaml, else the robot name)")
    args = parser.parse_args(argv)
    try:
        path = write_for_run(args.run_dir, args.out, args.title)
    except ValueError as error:
        print(f"report: {error}", file=sys.stderr)
        return 1
    print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
