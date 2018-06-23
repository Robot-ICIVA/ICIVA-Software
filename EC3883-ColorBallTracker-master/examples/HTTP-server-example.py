from socketserver import ThreadingMixIn
from http.server import SimpleHTTPRequestHandler, HTTPServer, BaseHTTPRequestHandler

class ThreadingSimpleServer(ThreadingMixIn, HTTPServer):
    pass

import sys
import os
import time
import json

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type','text/json')
        self.end_headers()

        t = [[4,5], "hola", 6,7]
        t_json = json.dumps(t)
        # self.wfile.write(t_json)
        self.wfile.write(bytes(t_json,"utf-8"))
        return

server = ThreadingSimpleServer(('', 8000), Handler)

try:
    while 1:
        start = time.time()
        # sys.stdout.flush()
        server.handle_request()
        stop = time.time()
        print("time = {}ms".format( (stop-start)/1000.0 ))
except KeyboardInterrupt:
    print("Finished")
