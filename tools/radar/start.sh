python3 -m venv .venv && source .venv/bin/activate   # optional
pip install -r requirements.txt

# Point at the machine running rosbridge if it's not localhost:
export RADAR_ROSBRIDGE_HOST=localhost
export RADAR_ROSBRIDGE_PORT=9090

uvicorn app:app --host 0.0.0.0 --port 8000