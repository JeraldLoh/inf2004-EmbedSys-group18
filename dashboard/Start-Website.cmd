@echo off
cd /d "%~dp0web"
echo Open http://localhost:8080 in Chrome or Edge.
echo Keep this window open while using the website. Press Ctrl+C to stop.
python -B -m http.server 8080 --bind 127.0.0.1
pause
