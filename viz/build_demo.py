"""Embed the experiment data in a self-contained, offline HTML replay."""

import argparse
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "results/experiment.json")
    parser.add_argument("--output", type=Path, default=ROOT / "results/demo.html")
    args = parser.parse_args()

    data = json.loads(args.input.read_text(encoding="utf-8"))
    if not data.get("scenarios"):
        parser.error("the experiment contains no scenarios")
    encoded = json.dumps(data, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    encoded = encoded.replace("<", "\\u003c")
    template = (Path(__file__).with_name("demo_template.html")).read_text(encoding="utf-8")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(template.replace("/* EXPERIMENT_JSON */", encoded), encoding="utf-8")
    print(f"Replay written to {args.output.name}")


if __name__ == "__main__":
    main()
