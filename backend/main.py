import json
import sys
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.concurrency import run_in_threadpool
from starlette.responses import FileResponse
from pydantic import BaseModel

import logging
from .log_handler import setup_logging

logger = setup_logging(drone_name="main_api")

app = FastAPI()
origins = [
    "http://localhost",
    "http://localhost:8080", # Если вы будете запускать HTML через какой-нибудь live server
    "http://127.0.0.1",
    "http://127.0.0.1:8000",
    "null"  # <-- ЭТО САМОЕ ВАЖНОЕ ДЛЯ ЛОКАЛЬНЫХ ФАЙЛОВ (file://)
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"], # Разрешить все методы (GET, POST и т.д.)
    allow_headers=["*"], # Разрешить все заголовки
)



if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "debug":
        mode = "ai"
        if len(sys.argv) > 2 and sys.argv[2] in ["ai", "alg"]:
            mode = sys.argv[2]
        # asyncio.run(debug_game_loop(mode))