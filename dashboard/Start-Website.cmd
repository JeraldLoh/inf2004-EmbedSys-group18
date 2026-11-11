@echo off
cd /d "%~dp0web"
echo Open http://localhost:8080 in Chrome or Edge.
echo Keep this window open while using the website. Press Ctrl+C to stop.
rem Windows can map .mjs to text/plain in the registry; browsers then refuse
rem the dashboard's JavaScript modules and no button works. Force the right type.
python -B -c "import http.server as h; h.SimpleHTTPRequestHandler.extensions_map['.mjs']='text/javascript'; h.test(HandlerClass=h.SimpleHTTPRequestHandler, port=8080, bind='127.0.0.1')"
pause
