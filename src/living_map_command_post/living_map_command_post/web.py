from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import List, Literal

from fastapi import FastAPI, WebSocket, HTTPException

from .demo import DemoError
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field


class MissionRequest(BaseModel):
    """WHO goes and WHAT the objective is. The robot decides HOW."""

    mission_id: int = Field(default=1, ge=1, le=65535)
    robot_id: str = Field(default="executor", min_length=1, max_length=32)
    # A specific memory to reach; when omitted the command post picks the most
    # urgent live memory of ``target_type``.
    target_beacon_id: int | None = Field(default=None, ge=1, le=65535)
    target_type: int = Field(default=1, ge=1, le=4)
    objective: str = Field(default="Reach the victim using inherited memories.", min_length=1, max_length=256)


class DemoRequest(BaseModel):
    action: Literal["full_demo", "start", "pause", "fail_writer", "stuck_writer", "destroy_beacon",
                    "dispatch", "cancel_auto"]
    outcome: Literal["destroyed", "stuck", "return"] | None = None
    beacon_id: int | None = Field(default=None, ge=1, le=65535)


class EventPosition(BaseModel):
    id: Literal["V01", "V02", "H01", "H02", "F01"]
    x: float = Field(allow_inf_nan=False)
    y: float = Field(allow_inf_nan=False)


class LinkRequest(BaseModel):
    up: bool
    restore_after_s: float | None = Field(default=None, gt=0, le=600)


class RestartRequest(BaseModel):
    mode: Literal["autonomous", "guided"] = "autonomous"
    speed: Literal["normal", "fast", "very_fast"] = "normal"
    events: List[EventPosition] | None = None


def _dashboard_dir() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("living_map_dashboard")) / "web"
        if path.exists():
            return path
    except Exception:
        pass
    source = Path(__file__).resolve().parents[2] / "living_map_dashboard" / "web"
    return source if source.exists() else None


def create_app(state, send_mission, demo=None, restart=None, scenario=None, set_link=None) -> FastAPI:
    app = FastAPI(title="LivingMap Command Post")
    web = _dashboard_dir()
    if web:
        app.mount("/static", StaticFiles(directory=str(web)), name="static")

    @app.get("/")
    def root():
        if web:
            return FileResponse(str(web / "index.html"))
        return JSONResponse({"service": "LivingMap Command Post", "dashboard": "assets not found"}, status_code=503)

    @app.get("/harness")
    def harness():
        """Simulation harness view: scenario setup, fault injection and link control."""
        if web:
            return FileResponse(str(web / "index.html"))
        return JSONResponse({"service": "LivingMap simulation harness", "dashboard": "assets not found"},
                            status_code=503)

    @app.post("/api/harness/link")
    def harness_link(payload: LinkRequest):
        if set_link is None:
            raise HTTPException(503, "Link control unavailable")
        if not set_link(payload.up, payload.restore_after_s):
            raise HTTPException(409, "Link control service is not ready")
        return {"accepted": True, "message": "Link restored" if payload.up else (
            f"Satellite link cut for {payload.restore_after_s:.0f} s" if payload.restore_after_s
            else "Satellite link cut until restored")}

    def snapshot():
        data = state.snapshot()
        if demo is not None:
            data["demo_control"] = demo.snapshot()
        return data

    @app.get("/api/state")
    def api_state():
        return snapshot()

    @app.post("/api/demo")
    def demo_action(payload: DemoRequest):
        if demo is None:
            raise HTTPException(503, "Demo controller unavailable")
        try:
            demo.request(payload.action, outcome=payload.outcome, beacon_id=payload.beacon_id)
        except DemoError as exc:
            raise HTTPException(409, str(exc)) from exc
        return {"accepted": True, "action": payload.action}

    @app.post("/api/demo/restart")
    def restart_demo(payload: RestartRequest):
        if restart is None:
            raise HTTPException(409, "Demo restart unavailable")
        try:
            if payload.events is None:
                accepted = restart(payload.mode, payload.speed)
            else:
                accepted = restart(payload.mode, payload.speed, [e.model_dump() for e in payload.events])
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc
        if not accepted:
            raise HTTPException(409, "Demo restart is unavailable or already pending")
        return {"accepted": True, "message": "Restarting the simulation. The dashboard will reconnect automatically."}

    @app.get("/api/scenario")
    def get_scenario():
        if scenario is None:
            raise HTTPException(503, "Scenario configuration unavailable")
        return scenario()


    @app.post("/api/mission")
    async def mission(payload: MissionRequest):
        data = payload.model_dump() if hasattr(payload, "model_dump") else payload.dict()
        try:
            accepted = bool(send_mission(data))
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc
        return {"accepted": accepted, "mission_id": payload.mission_id}

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        try:
            while True:
                await sock.send_text(json.dumps(snapshot()))
                await asyncio.sleep(0.5)
        except Exception:
            pass

    return app
