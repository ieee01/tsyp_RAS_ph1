from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import List

from fastapi import FastAPI, WebSocket
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field


class MissionRequest(BaseModel):
    mission_id: int = Field(default=1, ge=1, le=65535)
    target_event: str = Field(default="V01", min_length=1, max_length=32)
    target_type: int = Field(default=1, ge=1, le=4)
    avoid_event_types: List[int] = Field(default_factory=lambda: [2, 4])
    objective: str = Field(
        default="Reach Victim V01 while avoiding known hazards.",
        min_length=1,
        max_length=256,
    )


def _dashboard_dir() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("living_map_dashboard")) / "web"
        if installed.exists():
            return installed
    except Exception:
        pass

    # Source-tree fallback is useful for pure-Python tests only. Runtime does not
    # depend on the current working directory because the dashboard package is an
    # explicit gateway dependency.
    source = Path(__file__).resolve().parents[2] / "living_map_dashboard" / "web"
    return source if source.exists() else None


def create_app(state, send_mission) -> FastAPI:
    app = FastAPI(title="LivingMap Gateway API")
    web = _dashboard_dir()
    if web:
        app.mount("/static", StaticFiles(directory=str(web)), name="static")

    @app.get("/")
    def root():
        if web:
            return FileResponse(str(web / "index.html"))
        return JSONResponse({"service": "LivingMap Gateway", "dashboard": "assets not found"}, status_code=503)

    @app.get("/api/state")
    def api_state():
        return state.snapshot()

    @app.post("/api/mission")
    async def mission(payload: MissionRequest):
        data = payload.model_dump() if hasattr(payload, "model_dump") else payload.dict()
        accepted = bool(send_mission(data))
        return {"accepted": accepted, "mission_id": payload.mission_id}

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        try:
            while True:
                await sock.send_text(json.dumps(state.snapshot()))
                await asyncio.sleep(0.5)
        except Exception:
            # Client disconnects are expected and should not affect the ROS node.
            pass

    return app
