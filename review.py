"""Open saved telemetry runs in a browser without connecting to the robot."""

import argparse
from pathlib import Path

from src.config_loader import load_config
from src.run_review import RunStore, make_server


def main():
    config = load_config()
    parser = argparse.ArgumentParser(description="Review saved RoboMaster runs")
    parser.add_argument("--data-dir", type=Path, help="CSV run directory")
    parser.add_argument("--host", default=config["review"]["host"])
    parser.add_argument("--port", type=int, default=config["review"]["port"])
    args = parser.parse_args()

    project_dir = Path(__file__).resolve().parent
    data_dir = args.data_dir or project_dir / config["logging"]["directory"]
    store = RunStore(
        data_dir,
        config["review"]["max_points_per_stream"],
        config["review"]["close_tof_mm"],
    )
    server = make_server(store, args.host, args.port)
    print(f"Review dashboard: http://{args.host}:{args.port}")
    print(f"Reading runs from: {data_dir}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
