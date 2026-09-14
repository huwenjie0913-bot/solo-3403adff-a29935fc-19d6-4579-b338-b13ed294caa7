"""SQLite persistence: every computation is stored with its algorithm
version so any plan can be replayed by id."""
import json
import sqlite3
import threading
import uuid
from datetime import datetime, timezone

import numpy as np

from . import ALGO_VERSION

SCHEMA = """
CREATE TABLE IF NOT EXISTS plans (
    id           TEXT PRIMARY KEY,
    parent_id    TEXT,
    created_at   TEXT NOT NULL,
    algo_version TEXT NOT NULL,
    request_json TEXT NOT NULL,
    result_json  TEXT NOT NULL,
    geojson_json TEXT NOT NULL
);
"""


def _json_default(o):
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.integer):
        return int(o)
    raise TypeError(f"not JSON serializable: {type(o)!r}")


class Store:
    def __init__(self, path):
        self.path = path
        self._lock = threading.Lock()
        with self._connect() as con:
            con.executescript(SCHEMA)

    def _connect(self):
        con = sqlite3.connect(self.path)
        con.row_factory = sqlite3.Row
        return con

    def save(self, parent_id, request, result, geojson):
        pid = uuid.uuid4().hex[:12]
        with self._lock, self._connect() as con:
            con.execute(
                "INSERT INTO plans VALUES (?,?,?,?,?,?,?)",
                (
                    pid,
                    parent_id,
                    datetime.now(timezone.utc).isoformat(),
                    ALGO_VERSION,
                    json.dumps(request, default=_json_default),
                    json.dumps(result, default=_json_default),
                    json.dumps(geojson, default=_json_default),
                ),
            )
        return pid

    def get(self, pid):
        with self._connect() as con:
            row = con.execute("SELECT * FROM plans WHERE id = ?", (pid,)).fetchone()
        if row is None:
            return None
        return {
            "id": row["id"],
            "parent_id": row["parent_id"],
            "created_at": row["created_at"],
            "algo_version": row["algo_version"],
            "request": json.loads(row["request_json"]),
            "result": json.loads(row["result_json"]),
            "geojson": json.loads(row["geojson_json"]),
        }

    def list(self):
        with self._connect() as con:
            rows = con.execute(
                "SELECT id, parent_id, created_at, algo_version, result_json "
                "FROM plans ORDER BY created_at"
            ).fetchall()
        out = []
        for r in rows:
            result = json.loads(r["result_json"])
            out.append({
                "id": r["id"],
                "parent_id": r["parent_id"],
                "created_at": r["created_at"],
                "algo_version": r["algo_version"],
                "metrics": result.get("metrics"),
            })
        return out
